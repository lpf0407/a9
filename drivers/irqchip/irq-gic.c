/*
 *  linux/arch/arm/common/gic.c
 *
 *  Copyright (C) 2002 ARM Limited, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Interrupt architecture for the GIC:
 *
 * o There is one Interrupt Distributor, which receives interrupts
 *   from system devices and sends them to the Interrupt Controllers.
 *
 * o There is one CPU Interface per CPU, which sends interrupts sent
 *   by the Distributor, and interrupts generated locally, to the
 *   associated CPU. The base address of the CPU interface is usually
 *   aliased so that the same address points to different chips depending
 *   on the CPU it is accessed from.
 *
 * Note that IRQs 0-31 are special - they are local to each CPU.
 * As such, the enable set/clear, pending set/clear and active bit
 * registers are banked per-cpu for these sources.
 */
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/smp.h>
#include <linux/cpu.h>
#include <linux/cpu_pm.h>
#include <linux/cpumask.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/irqdomain.h>
#include <linux/interrupt.h>
#include <linux/percpu.h>
#include <linux/slab.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqchip/arm-gic.h>

#include <asm/cputype.h>
#include <asm/irq.h>
#include <asm/exception.h>
#include <asm/smp_plat.h>

#include "irqchip.h"

union gic_base {
	void __iomem *common_base;
	void __percpu * __iomem *percpu_base;
};

struct gic_chip_data {
	union gic_base dist_base;//GIC Distributor的基地址空间
	union gic_base cpu_base;//GIC CPU interface的基地址空间
#ifdef CONFIG_CPU_PM //GIC 电源管理相关的成员 
	u32 saved_spi_enable[DIV_ROUND_UP(1020, 32)];
	u32 saved_spi_conf[DIV_ROUND_UP(1020, 16)];
	u32 saved_spi_target[DIV_ROUND_UP(1020, 4)];
	u32 __percpu *saved_ppi_enable;
	u32 __percpu *saved_ppi_conf;
#endif
	struct irq_domain *domain;//该GIC对应的irq domain数据结构
	unsigned int gic_irqs;//GIC支持的IRQ的数目
	//对于GIC支持的IRQ的数目，这里还要赘述几句。实际上并非GIC支持
	//多少个HW interrupt ID，其就支持多少个IRQ。对于SGI，其处理比
	//较特别，并不归入IRQ number中。因此，对于GIC而言，其SGI（从0
	//到15的那些HW interrupt ID）不需要irq domain进行映射处理，也
	//就是说SGI没有对应的IRQ number。如果系统越来越复杂，一个GIC不i
	//能支持所有的interrupt source（目前GIC支持1020个中断源，这个数目
	//已经非常的大了），那么系统还需要引入secondary GIC，这个GIC主要负
	//责扩展外设相关的interrupt source，也就是说，secondary GIC的SGI和
	//PPI都变得冗余了（这些功能，primary GIC已经提供了）。这些信息可以
	//协助理解代码中的hwirq_base的设定。
#ifdef CONFIG_GIC_NON_BANKED
	void __iomem *(*get_base)(union gic_base *);
#endif
};

static DEFINE_RAW_SPINLOCK(irq_controller_lock);

/*
 * The GIC mapping of CPU interfaces does not necessarily match
 * the logical CPU numbering.  Let's use a mapping as returned
 * by the GIC itself.
 */
#define NR_GIC_CPU_IF 8
static u8 gic_cpu_map[NR_GIC_CPU_IF] __read_mostly;

/*
 * Supported arch specific GIC irq extension.
 * Default make them NULL.
 */
struct irq_chip gic_arch_extn = {
	.irq_eoi	= NULL,
	.irq_mask	= NULL,
	.irq_unmask	= NULL,
	.irq_retrigger	= NULL,
	.irq_set_type	= NULL,
	.irq_set_wake	= NULL,
};

#ifndef MAX_GIC_NR
#define MAX_GIC_NR	1
#endif

static struct gic_chip_data gic_data[MAX_GIC_NR] __read_mostly;

#ifdef CONFIG_GIC_NON_BANKED
static void __iomem *gic_get_percpu_base(union gic_base *base)
{
	return *__this_cpu_ptr(base->percpu_base);
}

static void __iomem *gic_get_common_base(union gic_base *base)
{
	return base->common_base;
}

static inline void __iomem *gic_data_dist_base(struct gic_chip_data *data)
{
	return data->get_base(&data->dist_base);
}

static inline void __iomem *gic_data_cpu_base(struct gic_chip_data *data)
{
	return data->get_base(&data->cpu_base);
}

static inline void gic_set_base_accessor(struct gic_chip_data *data,
					 void __iomem *(*f)(union gic_base *))
{
	data->get_base = f;
}
#else
#define gic_data_dist_base(d)	((d)->dist_base.common_base)
#define gic_data_cpu_base(d)	((d)->cpu_base.common_base)
#define gic_set_base_accessor(d, f)
#endif

static inline void __iomem *gic_dist_base(struct irq_data *d)
{
	struct gic_chip_data *gic_data = irq_data_get_irq_chip_data(d);
	return gic_data_dist_base(gic_data);
}

static inline void __iomem *gic_cpu_base(struct irq_data *d)
{
	struct gic_chip_data *gic_data = irq_data_get_irq_chip_data(d);
	return gic_data_cpu_base(gic_data);
}

static inline unsigned int gic_irq(struct irq_data *d)
{
	return d->hwirq;
}

/*
 * Routines to acknowledge, disable and enable interrupts
 */
static void gic_mask_irq(struct irq_data *d)
{
	u32 mask = 1 << (gic_irq(d) % 32);

	raw_spin_lock(&irq_controller_lock);
	writel_relaxed(mask, gic_dist_base(d) + GIC_DIST_ENABLE_CLEAR + (gic_irq(d) / 32) * 4);
	if (gic_arch_extn.irq_mask)
		gic_arch_extn.irq_mask(d);
	raw_spin_unlock(&irq_controller_lock);
}

static void gic_unmask_irq(struct irq_data *d)
{
	u32 mask = 1 << (gic_irq(d) % 32);

	raw_spin_lock(&irq_controller_lock);
	if (gic_arch_extn.irq_unmask)
		gic_arch_extn.irq_unmask(d);
	writel_relaxed(mask, gic_dist_base(d) + GIC_DIST_ENABLE_SET + (gic_irq(d) / 32) * 4);
	raw_spin_unlock(&irq_controller_lock);
}

static void gic_eoi_irq(struct irq_data *d)
{
	if (gic_arch_extn.irq_eoi) {
		raw_spin_lock(&irq_controller_lock);
		gic_arch_extn.irq_eoi(d);
		raw_spin_unlock(&irq_controller_lock);
	}

	writel_relaxed(gic_irq(d), gic_cpu_base(d) + GIC_CPU_EOI);
}

static int gic_set_type(struct irq_data *d, unsigned int type)
{
	void __iomem *base = gic_dist_base(d);
	unsigned int gicirq = gic_irq(d);
	u32 enablemask = 1 << (gicirq % 32);
	u32 enableoff = (gicirq / 32) * 4;
	u32 confmask = 0x2 << ((gicirq % 16) * 2);
	u32 confoff = (gicirq / 16) * 4;
	bool enabled = false;
	u32 val;

	/* Interrupt configuration for SGIs can't be changed */
	if (gicirq < 16)
		return -EINVAL;

	if (type != IRQ_TYPE_LEVEL_HIGH && type != IRQ_TYPE_EDGE_RISING)
		return -EINVAL;

	raw_spin_lock(&irq_controller_lock);

	if (gic_arch_extn.irq_set_type)
		gic_arch_extn.irq_set_type(d, type);

	val = readl_relaxed(base + GIC_DIST_CONFIG + confoff);
	if (type == IRQ_TYPE_LEVEL_HIGH)
		val &= ~confmask;
	else if (type == IRQ_TYPE_EDGE_RISING)
		val |= confmask;

	/*
	 * As recommended by the spec, disable the interrupt before changing
	 * the configuration
	 */
	if (readl_relaxed(base + GIC_DIST_ENABLE_SET + enableoff) & enablemask) {
		writel_relaxed(enablemask, base + GIC_DIST_ENABLE_CLEAR + enableoff);
		enabled = true;
	}

	writel_relaxed(val, base + GIC_DIST_CONFIG + confoff);

	if (enabled)
		writel_relaxed(enablemask, base + GIC_DIST_ENABLE_SET + enableoff);

	raw_spin_unlock(&irq_controller_lock);

	return 0;
}

static int gic_retrigger(struct irq_data *d)
{
	if (gic_arch_extn.irq_retrigger)
		return gic_arch_extn.irq_retrigger(d);

	/* the genirq layer expects 0 if we can't retrigger in hardware */
	return 0;
}

#ifdef CONFIG_SMP
static int gic_set_affinity(struct irq_data *d, const struct cpumask *mask_val,
			    bool force)
{
	void __iomem *reg = gic_dist_base(d) + GIC_DIST_TARGET + (gic_irq(d) & ~3);
	unsigned int cpu, shift = (gic_irq(d) % 4) * 8;
	u32 val, mask, bit;

	if (!force)
		cpu = cpumask_any_and(mask_val, cpu_online_mask);
	else
		cpu = cpumask_first(mask_val);

	if (cpu >= NR_GIC_CPU_IF || cpu >= nr_cpu_ids)
		return -EINVAL;

	raw_spin_lock(&irq_controller_lock);
	mask = 0xff << shift;
	bit = gic_cpu_map[cpu] << shift;
	val = readl_relaxed(reg) & ~mask;
	writel_relaxed(val | bit, reg);
	raw_spin_unlock(&irq_controller_lock);

	return IRQ_SET_MASK_OK;
}
#endif

#ifdef CONFIG_PM
static int gic_set_wake(struct irq_data *d, unsigned int on)
{
	int ret = -ENXIO;

	if (gic_arch_extn.irq_set_wake)
		ret = gic_arch_extn.irq_set_wake(d, on);

	return ret;
}

#else
#define gic_set_wake	NULL
#endif

static void __exception_irq_entry gic_handle_irq(struct pt_regs *regs)
{
	u32 irqstat, irqnr;
	struct gic_chip_data *gic = &gic_data[0];//获取root GIC的硬件描述符 
	void __iomem *cpu_base = gic_data_cpu_base(gic);//获取root GIC mapping到CPU地址空间的信息

	do {
		irqstat = readl_relaxed(cpu_base + GIC_CPU_INTACK);//获取HW interrupt ID
		irqnr = irqstat & GICC_IAR_INT_ID_MASK;

		if (likely(irqnr > 15 && irqnr < 1021)) {//SPI和PPI的处理
			irqnr = irq_find_mapping(gic->domain, irqnr);//将HW interrupt ID转成IRQ number 
			handle_IRQ(irqnr, regs);//处理该IRQ number
			continue;
		}
		if (irqnr < 16) {
			writel_relaxed(irqstat, cpu_base + GIC_CPU_EOI);//IPI类型的中断处理 
#ifdef CONFIG_SMP
			handle_IPI(irqnr, regs);
#endif
			continue;
		}
		break;
	} while (1);
}

static void gic_handle_cascade_irq(unsigned int irq, struct irq_desc *desc)
{
	struct gic_chip_data *chip_data = irq_get_handler_data(irq);
	struct irq_chip *chip = irq_get_chip(irq);
	unsigned int cascade_irq, gic_irq;
	unsigned long status;

	chained_irq_enter(chip, desc);

	raw_spin_lock(&irq_controller_lock);
	status = readl_relaxed(gic_data_cpu_base(chip_data) + GIC_CPU_INTACK);
	raw_spin_unlock(&irq_controller_lock);

	gic_irq = (status & 0x3ff);
	if (gic_irq == 1023)
		goto out;

	cascade_irq = irq_find_mapping(chip_data->domain, gic_irq);
	if (unlikely(gic_irq < 32 || gic_irq > 1020))
		handle_bad_irq(cascade_irq, desc);
	else
		generic_handle_irq(cascade_irq);

 out:
	chained_irq_exit(chip, desc);
}

static struct irq_chip gic_chip = {
	.name			= "GIC",
	.irq_mask		= gic_mask_irq,
	.irq_unmask		= gic_unmask_irq,
	.irq_eoi		= gic_eoi_irq,
	.irq_set_type		= gic_set_type,
	.irq_retrigger		= gic_retrigger,
#ifdef CONFIG_SMP
	.irq_set_affinity	= gic_set_affinity,
#endif
	.irq_set_wake		= gic_set_wake,
};

void __init gic_cascade_irq(unsigned int gic_nr, unsigned int irq)
{
	if (gic_nr >= MAX_GIC_NR)
		BUG();
	if (irq_set_handler_data(irq, &gic_data[gic_nr]) != 0)
		BUG();
	irq_set_chained_handler(irq, gic_handle_cascade_irq);
}

static u8 gic_get_cpumask(struct gic_chip_data *gic)
{
	void __iomem *base = gic_data_dist_base(gic);
	u32 mask, i;

	for (i = mask = 0; i < 32; i += 4) {
		mask = readl_relaxed(base + GIC_DIST_TARGET + i);
		//每个GIC上的interrupt ID都有8个bit来控制送达的target CPU
		//GIC_DIST_TARGETn（Interrupt Processor Targets Registers）
		//位于Distributor HW block中，能控制送达的CPU interface，
		//并不是具体的CPU，如果具体的实现中CPU interface和CPU是严
		//格按照上图中那样一一对应，那么GIC_DIST_TARGET送达了CPU Interface n，
		//也就是送达了CPU n。当然现实未必如你所愿，那么怎样来获取这个CPU的mask呢？
		//我们知道SGI和PPI不需要使用GIC_DIST_TARGET控制target CPU。SGI送达目标CPU
		//有自己特有的寄存器来控制（Software Generated Interrupt Register），对于
		//PPI，其是CPU私有的，因此不需要控制target CPU。GIC_DIST_TARGET0～GIC_DIST_TARGET7是
		//控制0～31这32个interrupt ID（SGI和PPI）的target CPU的，但是实际上SGI和PPI是不需要
		//控制target CPU的，因此，这些寄存器是read only的，读取这些寄存器返回的就是cpu mask值。
		//假设CPU0接在CPU interface 4上，那么运行在CPU 0上的程序在读GIC_DIST_TARGET0～GIC_DIST_TARGET7的时候，返回的就是0b00010000
		//
		mask |= mask >> 16;
		mask |= mask >> 8;
		if (mask)
			break;
	}

	if (!mask)
		pr_crit("GIC CPU mask not found - kernel will fail to boot.\n");

	return mask;
}

static void __init gic_dist_init(struct gic_chip_data *gic)
{
	unsigned int i;
	u32 cpumask;
	unsigned int gic_irqs = gic->gic_irqs;//获取该GIC支持的IRQ的数目 
	void __iomem *base = gic_data_dist_base(gic);//获取该GIC对应的Distributor基地址

	writel_relaxed(0, base + GIC_DIST_CTRL);
	//Distributor Control Register用来控制全局的中断forward情况。写入0表示Distributor
	//不向CPU interface发送中断请求信号，也就disable了全部的中断请求（group 0和group 1）
	//，CPU interace再也收不到中断请求信号了。在初始化的最后，step（f）那里会进行enable的动
	//作（这里只是enable了group 0的中断）。在初始化代码中，并没有设定interrupt source的group
	//（寄存器是GIC_DIST_IGROUP），我相信缺省值就是设定为group 0的。

	/*
	 * Set all global interrupts to be level triggered, active low.
	 */
	for (i = 32; i < gic_irqs; i += 16)
		writel_relaxed(0, base + GIC_DIST_CONFIG + i * 4 / 16);

	/*
	 * Set all global interrupts to this CPU only.
	 */

	cpumask = gic_get_cpumask(gic);
	cpumask |= cpumask << 8;
	cpumask |= cpumask << 16;
	for (i = 32; i < gic_irqs; i += 4)
		writel_relaxed(cpumask, base + GIC_DIST_TARGET + i * 4 / 4);

	/*
	 * Set priority on all global interrupts.
	 */
	for (i = 32; i < gic_irqs; i += 4)
		writel_relaxed(0xa0a0a0a0, base + GIC_DIST_PRI + i * 4 / 4);

	/*
	 * Disable all interrupts.  Leave the PPI and SGIs alone
	 * as these enables are banked registers.
	 */
	for (i = 32; i < gic_irqs; i += 32)
		writel_relaxed(0xffffffff, base + GIC_DIST_ENABLE_CLEAR + i * 4 / 32);

	writel_relaxed(1, base + GIC_DIST_CTRL);
}

static void gic_cpu_init(struct gic_chip_data *gic)
{
	void __iomem *dist_base = gic_data_dist_base(gic);
	void __iomem *base = gic_data_cpu_base(gic);
	unsigned int cpu_mask, cpu = smp_processor_id();
	int i;

	/*
	 * Get what the GIC says our CPU mask is.
	 */
	BUG_ON(cpu >= NR_GIC_CPU_IF);
	cpu_mask = gic_get_cpumask(gic);
	gic_cpu_map[cpu] = cpu_mask;

	/*
	 * Clear our mask from the other map entries in case they're
	 * still undefined.
	 */
	for (i = 0; i < NR_GIC_CPU_IF; i++)
		if (i != cpu)
			gic_cpu_map[i] &= ~cpu_mask;

	/*
	 * Deal with the banked PPI and SGI interrupts - disable all
	 * PPI interrupts, ensure all SGI interrupts are enabled.
	 */
	writel_relaxed(0xffff0000, dist_base + GIC_DIST_ENABLE_CLEAR);
	writel_relaxed(0x0000ffff, dist_base + GIC_DIST_ENABLE_SET);

	/*
	 * Set priority on PPI and SGI interrupts
	 */
	for (i = 0; i < 32; i += 4)
		writel_relaxed(0xa0a0a0a0, dist_base + GIC_DIST_PRI + i * 4 / 4);

	writel_relaxed(0xf0, base + GIC_CPU_PRIMASK);
	writel_relaxed(1, base + GIC_CPU_CTRL);
}

void gic_cpu_if_down(void)
{
	void __iomem *cpu_base = gic_data_cpu_base(&gic_data[0]);
	writel_relaxed(0, cpu_base + GIC_CPU_CTRL);
}

#ifdef CONFIG_CPU_PM
/*
 * Saves the GIC distributor registers during suspend or idle.  Must be called
 * with interrupts disabled but before powering down the GIC.  After calling
 * this function, no interrupts will be delivered by the GIC, and another
 * platform-specific wakeup source must be enabled.
 */
static void gic_dist_save(unsigned int gic_nr)
{
	unsigned int gic_irqs;
	void __iomem *dist_base;
	int i;

	if (gic_nr >= MAX_GIC_NR)
		BUG();

	gic_irqs = gic_data[gic_nr].gic_irqs;
	dist_base = gic_data_dist_base(&gic_data[gic_nr]);

	if (!dist_base)
		return;

	for (i = 0; i < DIV_ROUND_UP(gic_irqs, 16); i++)
		gic_data[gic_nr].saved_spi_conf[i] =
			readl_relaxed(dist_base + GIC_DIST_CONFIG + i * 4);

	for (i = 0; i < DIV_ROUND_UP(gic_irqs, 4); i++)
		gic_data[gic_nr].saved_spi_target[i] =
			readl_relaxed(dist_base + GIC_DIST_TARGET + i * 4);

	for (i = 0; i < DIV_ROUND_UP(gic_irqs, 32); i++)
		gic_data[gic_nr].saved_spi_enable[i] =
			readl_relaxed(dist_base + GIC_DIST_ENABLE_SET + i * 4);
}

/*
 * Restores the GIC distributor registers during resume or when coming out of
 * idle.  Must be called before enabling interrupts.  If a level interrupt
 * that occured while the GIC was suspended is still present, it will be
 * handled normally, but any edge interrupts that occured will not be seen by
 * the GIC and need to be handled by the platform-specific wakeup source.
 */
static void gic_dist_restore(unsigned int gic_nr)
{
	unsigned int gic_irqs;
	unsigned int i;
	void __iomem *dist_base;

	if (gic_nr >= MAX_GIC_NR)
		BUG();

	gic_irqs = gic_data[gic_nr].gic_irqs;
	dist_base = gic_data_dist_base(&gic_data[gic_nr]);

	if (!dist_base)
		return;

	writel_relaxed(0, dist_base + GIC_DIST_CTRL);

	for (i = 0; i < DIV_ROUND_UP(gic_irqs, 16); i++)
		writel_relaxed(gic_data[gic_nr].saved_spi_conf[i],
			dist_base + GIC_DIST_CONFIG + i * 4);

	for (i = 0; i < DIV_ROUND_UP(gic_irqs, 4); i++)
		writel_relaxed(0xa0a0a0a0,
			dist_base + GIC_DIST_PRI + i * 4);

	for (i = 0; i < DIV_ROUND_UP(gic_irqs, 4); i++)
		writel_relaxed(gic_data[gic_nr].saved_spi_target[i],
			dist_base + GIC_DIST_TARGET + i * 4);

	for (i = 0; i < DIV_ROUND_UP(gic_irqs, 32); i++)
		writel_relaxed(gic_data[gic_nr].saved_spi_enable[i],
			dist_base + GIC_DIST_ENABLE_SET + i * 4);

	writel_relaxed(1, dist_base + GIC_DIST_CTRL);
}

static void gic_cpu_save(unsigned int gic_nr)
{
	int i;
	u32 *ptr;
	void __iomem *dist_base;
	void __iomem *cpu_base;

	if (gic_nr >= MAX_GIC_NR)
		BUG();

	dist_base = gic_data_dist_base(&gic_data[gic_nr]);
	cpu_base = gic_data_cpu_base(&gic_data[gic_nr]);

	if (!dist_base || !cpu_base)
		return;

	ptr = __this_cpu_ptr(gic_data[gic_nr].saved_ppi_enable);
	for (i = 0; i < DIV_ROUND_UP(32, 32); i++)
		ptr[i] = readl_relaxed(dist_base + GIC_DIST_ENABLE_SET + i * 4);

	ptr = __this_cpu_ptr(gic_data[gic_nr].saved_ppi_conf);
	for (i = 0; i < DIV_ROUND_UP(32, 16); i++)
		ptr[i] = readl_relaxed(dist_base + GIC_DIST_CONFIG + i * 4);

}

static void gic_cpu_restore(unsigned int gic_nr)
{
	int i;
	u32 *ptr;
	void __iomem *dist_base;
	void __iomem *cpu_base;

	if (gic_nr >= MAX_GIC_NR)
		BUG();

	dist_base = gic_data_dist_base(&gic_data[gic_nr]);
	cpu_base = gic_data_cpu_base(&gic_data[gic_nr]);

	if (!dist_base || !cpu_base)
		return;

	ptr = __this_cpu_ptr(gic_data[gic_nr].saved_ppi_enable);
	for (i = 0; i < DIV_ROUND_UP(32, 32); i++)
		writel_relaxed(ptr[i], dist_base + GIC_DIST_ENABLE_SET + i * 4);

	ptr = __this_cpu_ptr(gic_data[gic_nr].saved_ppi_conf);
	for (i = 0; i < DIV_ROUND_UP(32, 16); i++)
		writel_relaxed(ptr[i], dist_base + GIC_DIST_CONFIG + i * 4);

	for (i = 0; i < DIV_ROUND_UP(32, 4); i++)
		writel_relaxed(0xa0a0a0a0, dist_base + GIC_DIST_PRI + i * 4);

	writel_relaxed(0xf0, cpu_base + GIC_CPU_PRIMASK);
	writel_relaxed(1, cpu_base + GIC_CPU_CTRL);
}

static int gic_notifier(struct notifier_block *self, unsigned long cmd,	void *v)
{
	int i;

	for (i = 0; i < MAX_GIC_NR; i++) {
#ifdef CONFIG_GIC_NON_BANKED
		/* Skip over unused GICs */
		if (!gic_data[i].get_base)
			continue;
#endif
		switch (cmd) {
		case CPU_PM_ENTER:
			gic_cpu_save(i);
			break;
		case CPU_PM_ENTER_FAILED:
		case CPU_PM_EXIT:
			gic_cpu_restore(i);
			break;
		case CPU_CLUSTER_PM_ENTER:
			gic_dist_save(i);
			break;
		case CPU_CLUSTER_PM_ENTER_FAILED:
		case CPU_CLUSTER_PM_EXIT:
			gic_dist_restore(i);
			break;
		}
	}

	return NOTIFY_OK;
}

static struct notifier_block gic_notifier_block = {
	.notifier_call = gic_notifier,
};

static void __init gic_pm_init(struct gic_chip_data *gic)
{
	gic->saved_ppi_enable = __alloc_percpu(DIV_ROUND_UP(32, 32) * 4,
		sizeof(u32));
	BUG_ON(!gic->saved_ppi_enable);

	gic->saved_ppi_conf = __alloc_percpu(DIV_ROUND_UP(32, 16) * 4,
		sizeof(u32));
	BUG_ON(!gic->saved_ppi_conf);
//主要是分配两个per cpu的内存。这些内存在系统进入sleep状态的时候保存PPI
//的寄存器状态信息，在resume的时候，写回寄存器。对于root GIC，需要注册
//一个和电源管理的事件通知callback函数。不得不吐槽一下gic_notifier_block和gic_notifier
//这两个符号的命名，看不出来和电源管理有任何关系。更优雅的名字应该包括pm这样的符号，
//以便让其他工程师看到名字就立刻知道是和电源管理相关的。
	if (gic == &gic_data[0])
		cpu_pm_register_notifier(&gic_notifier_block);
}
#else
static void __init gic_pm_init(struct gic_chip_data *gic)
{
}
#endif

#ifdef CONFIG_SMP
static void gic_raise_softirq(const struct cpumask *mask, unsigned int irq)
{
	int cpu;
	unsigned long flags, map = 0;

	raw_spin_lock_irqsave(&irq_controller_lock, flags);

	/* Convert our logical CPU mask into a physical one. */
	for_each_cpu(cpu, mask)
		map |= gic_cpu_map[cpu];

	/*
	 * Ensure that stores to Normal memory are visible to the
	 * other CPUs before they observe us issuing the IPI.
	 */
	dmb(ishst);

	/* this always happens on GIC0 */
	writel_relaxed(map << 16 | irq, gic_data_dist_base(&gic_data[0]) + GIC_DIST_SOFTINT);

	raw_spin_unlock_irqrestore(&irq_controller_lock, flags);
}
#endif

#ifdef CONFIG_BL_SWITCHER
/*
 * gic_send_sgi - send a SGI directly to given CPU interface number
 *
 * cpu_id: the ID for the destination CPU interface
 * irq: the IPI number to send a SGI for
 */
void gic_send_sgi(unsigned int cpu_id, unsigned int irq)
{
	BUG_ON(cpu_id >= NR_GIC_CPU_IF);
	cpu_id = 1 << cpu_id;
	/* this always happens on GIC0 */
	writel_relaxed((cpu_id << 16) | irq, gic_data_dist_base(&gic_data[0]) + GIC_DIST_SOFTINT);
}

/*
 * gic_get_cpu_id - get the CPU interface ID for the specified CPU
 *
 * @cpu: the logical CPU number to get the GIC ID for.
 *
 * Return the CPU interface ID for the given logical CPU number,
 * or -1 if the CPU number is too large or the interface ID is
 * unknown (more than one bit set).
 */
int gic_get_cpu_id(unsigned int cpu)
{
	unsigned int cpu_bit;

	if (cpu >= NR_GIC_CPU_IF)
		return -1;
	cpu_bit = gic_cpu_map[cpu];
	if (cpu_bit & (cpu_bit - 1))
		return -1;
	return __ffs(cpu_bit);
}

/*
 * gic_migrate_target - migrate IRQs to another CPU interface
 *
 * @new_cpu_id: the CPU target ID to migrate IRQs to
 *
 * Migrate all peripheral interrupts with a target matching the current CPU
 * to the interface corresponding to @new_cpu_id.  The CPU interface mapping
 * is also updated.  Targets to other CPU interfaces are unchanged.
 * This must be called with IRQs locally disabled.
 */
void gic_migrate_target(unsigned int new_cpu_id)
{
	unsigned int cur_cpu_id, gic_irqs, gic_nr = 0;
	void __iomem *dist_base;
	int i, ror_val, cpu = smp_processor_id();
	u32 val, cur_target_mask, active_mask;

	if (gic_nr >= MAX_GIC_NR)
		BUG();

	dist_base = gic_data_dist_base(&gic_data[gic_nr]);
	if (!dist_base)
		return;
	gic_irqs = gic_data[gic_nr].gic_irqs;

	cur_cpu_id = __ffs(gic_cpu_map[cpu]);
	cur_target_mask = 0x01010101 << cur_cpu_id;
	ror_val = (cur_cpu_id - new_cpu_id) & 31;

	raw_spin_lock(&irq_controller_lock);

	/* Update the target interface for this logical CPU */
	gic_cpu_map[cpu] = 1 << new_cpu_id;

	/*
	 * Find all the peripheral interrupts targetting the current
	 * CPU interface and migrate them to the new CPU interface.
	 * We skip DIST_TARGET 0 to 7 as they are read-only.
	 */
	for (i = 8; i < DIV_ROUND_UP(gic_irqs, 4); i++) {
		val = readl_relaxed(dist_base + GIC_DIST_TARGET + i * 4);
		active_mask = val & cur_target_mask;
		if (active_mask) {
			val &= ~active_mask;
			val |= ror32(active_mask, ror_val);
			writel_relaxed(val, dist_base + GIC_DIST_TARGET + i*4);
		}
	}

	raw_spin_unlock(&irq_controller_lock);

	/*
	 * Now let's migrate and clear any potential SGIs that might be
	 * pending for us (cur_cpu_id).  Since GIC_DIST_SGI_PENDING_SET
	 * is a banked register, we can only forward the SGI using
	 * GIC_DIST_SOFTINT.  The original SGI source is lost but Linux
	 * doesn't use that information anyway.
	 *
	 * For the same reason we do not adjust SGI source information
	 * for previously sent SGIs by us to other CPUs either.
	 */
	for (i = 0; i < 16; i += 4) {
		int j;
		val = readl_relaxed(dist_base + GIC_DIST_SGI_PENDING_SET + i);
		if (!val)
			continue;
		writel_relaxed(val, dist_base + GIC_DIST_SGI_PENDING_CLEAR + i);
		for (j = i; j < i + 4; j++) {
			if (val & 0xff)
				writel_relaxed((1 << (new_cpu_id + 16)) | j,
						dist_base + GIC_DIST_SOFTINT);
			val >>= 8;
		}
	}
}

/*
 * gic_get_sgir_physaddr - get the physical address for the SGI register
 *
 * REturn the physical address of the SGI register to be used
 * by some early assembly code when the kernel is not yet available.
 */
static unsigned long gic_dist_physaddr;

unsigned long gic_get_sgir_physaddr(void)
{
	if (!gic_dist_physaddr)
		return 0;
	return gic_dist_physaddr + GIC_DIST_SOFTINT;
}

void __init gic_init_physaddr(struct device_node *node)
{
	struct resource res;
	if (of_address_to_resource(node, 0, &res) == 0) {
		gic_dist_physaddr = res.start;
		pr_info("GIC physical location is %#lx\n", gic_dist_physaddr);
	}
}

#else
#define gic_init_physaddr(node)  do { } while (0)
#endif
//建IRQ number和GIC hw interrupt ID之间映射关系的时候，需要调用该回调函数
static int gic_irq_domain_map(struct irq_domain *d, unsigned int irq,
				irq_hw_number_t hw)
{
	if (hw < 32) {//PPI and SGI
		//分配per cpu的内存，设定一些per cpu的flag。
		irq_set_percpu_devid(irq);
		//设定该中断描述符的irq chip和high level的handler
		irq_set_chip_and_handler(irq, &gic_chip,
					 handle_percpu_devid_irq);
		//设定irq flag是有效的（因为已经设定好了chip和handler了），并且request后不是auto enable的
		set_irq_flags(irq, IRQF_VALID | IRQF_NOAUTOEN);
	} else {
		//对于SPI，设定的high level irq event handler是handle_fasteoi_irq。对于SPI，是可以probe，并且request后是auto enable的
		irq_set_chip_and_handler(irq, &gic_chip,
					 handle_fasteoi_irq);
		set_irq_flags(irq, IRQF_VALID | IRQF_PROBE);

		//有些SOC会在各种外设中断和GIC之间增加cross bar（例如TI的OMAP芯片），这里是为那些ARM SOC准备的
		gic_routable_irq_domain_ops->map(d, irq, hw);
	}
	//设定irq chip的私有数据 
	irq_set_chip_data(irq, d->host_data);
	return 0;
}

static void gic_irq_domain_unmap(struct irq_domain *d, unsigned int irq)
{
	gic_routable_irq_domain_ops->unmap(d, irq);
}
//除了标准的属性之外，各个具体的interrupt controller可以定义自己的device binding
//些device bindings都需在irq chip driver这个层面进行解析。要给定某个外设的device tree node 和interrupt specifier，该函数可以解码出该设备使用的hw interrupt ID和linux irq type value 
static int gic_irq_domain_xlate(struct irq_domain *d,
				struct device_node *controller,
				const u32 *intspec, unsigned int intsize,
				unsigned long *out_hwirq, unsigned int *out_type)
{
	unsigned long ret = 0;

	if (d->of_node != controller)
		return -EINVAL;
	if (intsize < 3)
		return -EINVAL;

	/* Get the interrupt number and add 16 to skip over SGIs */
	*out_hwirq = intspec[1] + 16;

	/* For SPIs, we need to add 16 more to get the GIC irq ID number */
	if (!intspec[0]) {
		ret = gic_routable_irq_domain_ops->xlate(d, controller,
							 intspec,
							 intsize,
							 out_hwirq,
							 out_type);

		if (IS_ERR_VALUE(ret))
			return ret;
	}

	*out_type = intspec[2] & IRQ_TYPE_SENSE_MASK;

	return ret;
}

#ifdef CONFIG_SMP
static int gic_secondary_init(struct notifier_block *nfb, unsigned long action,
			      void *hcpu)
{
	if (action == CPU_STARTING || action == CPU_STARTING_FROZEN)
		gic_cpu_init(&gic_data[0]);
	return NOTIFY_OK;
}

/*
 * Notifier for enabling the GIC CPU interface. Set an arbitrarily high
 * priority because the GIC needs to be up before the ARM generic timers.
 */
static struct notifier_block gic_cpu_notifier = {
	.notifier_call = gic_secondary_init,
	.priority = 100,
};
#endif

static const struct irq_domain_ops gic_irq_domain_ops = {
	.map = gic_irq_domain_map,
	.unmap = gic_irq_domain_unmap,
	.xlate = gic_irq_domain_xlate,
};

/* Default functions for routable irq domain */
static int gic_routable_irq_domain_map(struct irq_domain *d, unsigned int irq,
			      irq_hw_number_t hw)
{
	return 0;
}

static void gic_routable_irq_domain_unmap(struct irq_domain *d,
					  unsigned int irq)
{
}

static int gic_routable_irq_domain_xlate(struct irq_domain *d,
				struct device_node *controller,
				const u32 *intspec, unsigned int intsize,
				unsigned long *out_hwirq,
				unsigned int *out_type)
{
	*out_hwirq += 16;
	return 0;
}

const struct irq_domain_ops gic_default_routable_irq_domain_ops = {
	.map = gic_routable_irq_domain_map,
	.unmap = gic_routable_irq_domain_unmap,
	.xlate = gic_routable_irq_domain_xlate,
};

const struct irq_domain_ops *gic_routable_irq_domain_ops =
					&gic_default_routable_irq_domain_ops;

void __init gic_init_bases(unsigned int gic_nr, int irq_start,
			   void __iomem *dist_base, void __iomem *cpu_base,
			   u32 percpu_offset, struct device_node *node)
{
	irq_hw_number_t hwirq_base;
	struct gic_chip_data *gic;
	int gic_irqs, irq_base, i;
	int nr_routable_irqs;

	BUG_ON(gic_nr >= MAX_GIC_NR);

	//gic_nr标识GIC number，等于0就是root GIC。hwirq的意思就是GIC上的HW interrupt ID，
	//并不是GIC上的每个interrupt ID都有map到linux IRQ framework中的一个IRQ number，
	gic = &gic_data[gic_nr];
#ifdef CONFIG_GIC_NON_BANKED
	if (percpu_offset) { /* Frankein-GIC without banked registers... */
		unsigned int cpu;

		gic->dist_base.percpu_base = alloc_percpu(void __iomem *);
		gic->cpu_base.percpu_base = alloc_percpu(void __iomem *);
		if (WARN_ON(!gic->dist_base.percpu_base ||
			    !gic->cpu_base.percpu_base)) {
			free_percpu(gic->dist_base.percpu_base);
			free_percpu(gic->cpu_base.percpu_base);
			return;
		}

		for_each_possible_cpu(cpu) {
			u32 mpidr = cpu_logical_map(cpu);
			u32 core_id = MPIDR_AFFINITY_LEVEL(mpidr, 0);
			unsigned long offset = percpu_offset * core_id;
			*per_cpu_ptr(gic->dist_base.percpu_base, cpu) = dist_base + offset;
			*per_cpu_ptr(gic->cpu_base.percpu_base, cpu) = cpu_base + offset;
		}

		gic_set_base_accessor(gic, gic_get_percpu_base);
	} else
#endif
	{			/* Normal, sane GIC... */
		WARN(percpu_offset,
		     "GIC_NON_BANKED not enabled, ignoring %08x offset!",
		     percpu_offset);
		gic->dist_base.common_base = dist_base;
		gic->cpu_base.common_base = cpu_base;
		gic_set_base_accessor(gic, gic_get_common_base);
	}

	/*
	 * Initialize the CPU interface map to all CPUs.
	 * It will be refined as each CPU probes its ID.
	 */
	for (i = 0; i < NR_GIC_CPU_IF; i++)
		gic_cpu_map[i] = 0xff;

	/*
	 * For primary GICs, skip over SGIs.
	 * For secondary GICs, skip over PPIs, too.
	 */
	if (gic_nr == 0 && (irq_start & 31) > 0) {
		hwirq_base = 16;
		if (irq_start != -1)
			irq_start = (irq_start & ~31) + 16;
	} else {
		hwirq_base = 32;
	}
	//对于SGI，是属于软件中断，用于CPU之间通信，没有必要进行HW interrupt ID到IRQ number的mapping。
	//变量hwirq_base表示该GIC上要进行map的base ID，hwirq_base = 16也就意味着忽略掉16个SGI。
	//对于系统中其他的GIC，其PPI也没有必要mapping，因此hwirq_base = 32。
	//在本场景中，irq_start ＝ -1，表示不指定IRQ number。有些场景会指定IRQ number，这时候，需要对IRQ number进行一个对齐的操作

	/*
	 * Find out how many interrupts are supported.
	 * The GIC only supports up to 1020 interrupt sources.
	 */
	gic_irqs = readl_relaxed(gic_data_dist_base(gic) + GIC_DIST_CTR) & 0x1f;
	gic_irqs = (gic_irqs + 1) * 32;
	//变量gic_irqs保存了该GIC支持的最大的中断数目。该信息是从GIC_DIST_CTR寄存器（这是V1版本的寄存器名字，V2中是GICD_TYPER，
	//Interrupt Controller Type Register,）的低五位ITLinesNumber获取的。如果ITLinesNumber等于N，那么最大支持的中断数目是32(N+1)。
	//此外，GIC规范规定最大的中断数目不能超过1020，1020-1023是有特别用户的interrupt ID
	if (gic_irqs > 1020)
		gic_irqs = 1020;
	gic->gic_irqs = gic_irqs;

	gic_irqs -= hwirq_base; /* calculate # of irqs to allocate */
	//减去不需要map（不需要分配IRQ）的那些interrupt ID，OK，这时候gic_irqs的数值终于和它的名字一致了。gic_irqs从字面上看不就是该GIC需要分配的IRQ number的数目吗

	//of_property_read_u32函数把arm,routable-irqs的属性值读出到nr_routable_irqs变量中，如果正确返回0。
	//在有些SOC的设计中，外设的中断请求信号线不是直接接到GIC，而是通过crossbar/multiplexer这个的HW block连接到GIC上。
	//arm,routable-irqs这个属性用来定义那些不直接连接到GIC的中断请求数目
	if (of_property_read_u32(node, "arm,routable-irqs",
				 &nr_routable_irqs)) {
		irq_base = irq_alloc_descs(irq_start, 16, gic_irqs,
					   numa_node_id());
		//对于那些直接连接到GIC的情况，我们需要通过调用irq_alloc_descs分配中断描述符。如果irq_start大于0，
		//那么说明是指定IRQ number的分配，对于我们这个场景，irq_start等于-1，因此不指定IRQ 号。如果不指定
		//IRQ number的，就需要搜索，第二个参数16就是起始搜索的IRQ number。gic_irqs指明要分配的irq number的
		//数目。如果没有正确的分配到中断描述符，程序会认为可能是之前已经准备好了
		if (IS_ERR_VALUE(irq_base)) {
			WARN(1, "Cannot allocate irq_descs @ IRQ%d, assuming pre-allocated\n",
			     irq_start);
			irq_base = irq_start;
		}

		gic->domain = irq_domain_add_legacy(node, gic_irqs, irq_base,
					hwirq_base, &gic_irq_domain_ops, gic);
	} else {
		gic->domain = irq_domain_add_linear(node, nr_routable_irqs,
						    &gic_irq_domain_ops,
						    gic);
	}
	//这段代码主要是向系统中注册一个irq domain的数据结构。为何需要struct irq_domain这样一个数据结构呢？
	//从linux kernel的角度来看，任何外部的设备的中断都是一个异步事件，kernel都需要识别这个事件。在内核中，
	//用IRQ number来标识某一个设备的某个interrupt request。有了IRQ number就可以定位到该中断的描述符（struct irq_desc）。
	//但是，对于中断控制器而言，它不并知道IRQ number，它只是知道HW interrupt number（中断控制器会为其支持的interrupt source
	//进行编码，这个编码被称为Hardware interrupt number ）。不同的软件模块用不同的ID来识别interrupt source，这样就需要映射了。
	//如何将Hardware interrupt number 映射到IRQ number呢？这需要一个translation object，内核定义为struct irq_domain。

	//每个interrupt controller都会形成一个irq domain，负责解析其下游的interrut source。如果interrupt controller有级联的情况，
	//那么一个非root interrupt controller的中断控制器也是其parent irq domain的一个普通的interrupt source
	//
	//
	//irq domain的概念是一个通用中断子系统的概念，在具体的irq chip driver这个层次，我们需要一些解析GIC binding，
	//创建IRQ number和HW interrupt ID的mapping的callback函数，
	if (WARN_ON(!gic->domain))
		return;

	if (gic_nr == 0) {
#ifdef CONFIG_SMP
		set_smp_cross_call(gic_raise_softirq);
		//set_smp_cross_call这个函数看名字也知道它的含义，就是设定一个多个CPU直接通信的callback函数。当一个CPU core上的软
		//件控制行为需要传递到其他的CPU上的时候（例如在某一个CPU上运行的进程调用了系统调用进行reboot），就会调用这个callback函
		//数。对于GIC，这个callback定义为gic_raise_softirq。这个函数名字起的不好，直观上以为是和softirq相关，实际上其实是触发了IPI中断
		register_cpu_notifier(&gic_cpu_notifier);
		//在multi processor环境下，当processor状态发送变化的时候（例如online，offline），需要把这些事件通知到GIC。而GIC driver在收到
		//来自CPU的事件后会对cpu interface进行相应的设定
#endif
		set_handle_irq(gic_handle_irq);//设定arch相关的irq handler
	}

	gic_chip.flags |= gic_arch_extn.flags;
	gic_dist_init(gic);//具体硬件初始化
	gic_cpu_init(gic);
	gic_pm_init(gic);
}

#ifdef CONFIG_OF
static int gic_cnt __initdata;

static int __init
gic_of_init(struct device_node *node, struct device_node *parent)
{
	//node参数代表需要初始化的那个interrupt controller的device node，parent参数指向其parent
	void __iomem *cpu_base;
	void __iomem *dist_base;
	u32 percpu_offset;
	int irq;

	if (WARN_ON(!node))
		return -ENODEV;

	dist_base = of_iomap(node, 0);//映射GIC Distributor的寄存器地址空间
	WARN(!dist_base, "unable to map gic dist registers\n");

	cpu_base = of_iomap(node, 1);//映射GIC CPU interface的寄存器地址空间
	WARN(!cpu_base, "unable to map gic cpu registers\n");

	if (of_property_read_u32(node, "cpu-offset", &percpu_offset))//处理cpu-offset属性
		percpu_offset = 0;
	//cpu-offset属性，首先要了解什么是banked register。所谓banked register就是在一个地
	//址上提供多个寄存器副本。比如说系统中有四个CPU，这些CPU访问某个寄存器的时候地址是
	//一样的，但是对于banked register，实际上，不同的CPU访问的是不同的寄存器，虽然它们
	//的地址是一样的。如果GIC没有banked register，那么需要提供根据CPU index给出一系列
	//地址偏移，而地址偏移=cpu-offset * cpu-nr

	gic_init_bases(gic_cnt, -1, dist_base, cpu_base, percpu_offset, node);//主处理过程，后面详述
	if (!gic_cnt)
		gic_init_physaddr(node);//对于不支持big.LITTLE switcher（CONFIG_BL_SWITCHER）的系统，该函数为空。

	if (parent) {//处理interrupt级联 
		irq = irq_of_parse_and_map(node, 0);//解析second GIC的interrupts属性，并进行mapping，返回IRQ number 
		gic_cascade_irq(gic_cnt, irq);
		//interrupt controller可以级联。对于root GIC，其传入的parent是NULL，因此不会执行级联部分的代码。
		//对于second GIC，它是作为其parent（root GIC）的一个普通的irq source，因此，也需要注册该IRQ的handler。
		//由此可见，非root的GIC的初始化分成了两个部分：一部分是作为一个interrupt controller，执行和root GIC一
		//样的初始化代码。另外一方面，GIC又作为一个普通的interrupt generating device，需要象一个普通的设备驱动
		//一样，注册其中断handler。理解irq_of_parse_and_map需要irq domain的知识，
	}
	gic_cnt++;
	return 0;
}
IRQCHIP_DECLARE(gic_400, "arm,gic-400", gic_of_init);
IRQCHIP_DECLARE(cortex_a15_gic, "arm,cortex-a15-gic", gic_of_init);
IRQCHIP_DECLARE(cortex_a9_gic, "arm,cortex-a9-gic", gic_of_init);
IRQCHIP_DECLARE(cortex_a7_gic, "arm,cortex-a7-gic", gic_of_init);
IRQCHIP_DECLARE(msm_8660_qgic, "qcom,msm-8660-qgic", gic_of_init);
IRQCHIP_DECLARE(msm_qgic2, "qcom,msm-qgic2", gic_of_init);

#endif
