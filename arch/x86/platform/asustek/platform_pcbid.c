/*
 * Copyright (C) 2012 ASUSTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/bitops.h>
#include <linux/gpio.h>
#include <linux/string.h>
#include <linux/sfi.h>
#include <linux/HWVersion.h>

#ifdef CONFIG_FE170CG
#include <linux/lnw_gpio.h>
#include <linux/delay.h>
#include <asm/intel_scu_pmic.h>

extern void intel_mid_set_pcb_id(int value);
extern void intel_mid_set_rc_version(int value);
extern void intel_mid_set_sim_id(int value);
extern void intel_mid_set_voice_id(int value);
extern void intel_mid_set_project_id(int value);
extern void intel_mid_set_hardware_id(int value);
extern void intel_mid_set_rf_id(int value);
#endif

//Chipang add for detect platform ID ++
static int PROJECT_ID;
static int HARDWARE_ID;
static int PCB_ID;
static int TP_ID;
static int RC_VERSION;
static int RF_ID;
static int rc_version;
//Chipang add for detect platform ID --

module_param(rc_version, int, S_IRUGO | S_IWUSR);
int Read_RC_VERSION(void)
{
     rc_version = RC_VERSION;
     printk("RC_VERSION = 0x%x \n",RC_VERSION);
     return RC_VERSION;
}
EXPORT_SYMBOL(Read_RC_VERSION);

#ifdef CONFIG_FE170CG
static int voice_id;
module_param(voice_id, int, S_IRUGO | S_IWUSR);
static int sim_id = 0;
module_param(sim_id, int, S_IRUGO | S_IWUSR);
#define SIM_DETECT_GPIO_PIN 118

int read_SIM_ID(void){
	// get the PCB_ID11 value
	gpio_request(SIM_DETECT_GPIO_PIN, "sim_detect");
	lnw_gpio_set_alt(SIM_DETECT_GPIO_PIN, 0);
	usleep_range(10, 20);
	gpio_direction_input(SIM_DETECT_GPIO_PIN);
	sim_id = (gpio_get_value_cansleep(SIM_DETECT_GPIO_PIN) > 0) ? 1 : 0;
	PCB_ID &= 0xF7FF;
	PCB_ID |= sim_id << 11; 
	printk("SIM detect value is %d\n", sim_id);
	return sim_id;
}
EXPORT_SYMBOL(read_SIM_ID);


	
#define PMIC_GPIO1HV2CTLO 0x72
#define PMIC_GPIO1HV2CTLI 0x7A
void initialize_VOICE_ID(){
	char data;
	intel_scu_ipc_iowrite8(PMIC_GPIO1HV2CTLO, 0x0C);
	msleep(1);
	intel_scu_ipc_ioread8(PMIC_GPIO1HV2CTLI,&data);
	intel_scu_ipc_iowrite8(PMIC_GPIO1HV2CTLO, 0x08);
	voice_id = data & 0x01;
	printk("voice_id = %d\n", voice_id);
	return voice_id;
}
EXPORT_SYMBOL(initialize_VOICE_ID);

int read_VOICE_ID(){
	printk("voice_id = %d\n", voice_id);
	return voice_id;
}
EXPORT_SYMBOL(read_VOICE_ID);

static int rf_id;
module_param(rf_id, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(HW_VERSION,
		 "RF_ID judgement");

int Read_RF_ID(void)
{
	if(PROJECT_ID == 0x07){
		printk("RF_ID = 0x%x but change to 2 for JP device\n",RF_ID);
	    rf_id = 2;
	}else{
	    printk("RF_ID = 0x%x \n",RF_ID);
	    rf_id = RF_ID;
	}
	return RF_ID;
}
EXPORT_SYMBOL(Read_RF_ID);
#endif

#ifdef CONFIG_TF103CG
static int RC_VERSION;
#endif

//Thomas add for ATD Tool atd_pcb_id {
static int atd_pcb_id;
module_param(atd_pcb_id, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(ATD_PCB_ID_VERSION,
                 "ATD_PCB_ID judgement");
//Thomas add for ATD Tool atd_pcb_id }

static int project_id;
module_param(project_id, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(PROJ_VERSION,
                 "PROJ_ID judgement");

int Read_PROJ_ID(void)
{
        pr_debug("PROJECT_ID = 0x%x \n",PROJECT_ID);
        project_id = PROJECT_ID;
        return PROJECT_ID;
}
EXPORT_SYMBOL(Read_PROJ_ID);

static int hardware_id;
module_param(hardware_id, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(HW_VERSION,
                 "HW_ID judgement");

int Read_HW_ID(void)
{
        pr_debug("HARDWARE_ID = 0x%x \n",HARDWARE_ID);
        hardware_id = HARDWARE_ID;
        return HARDWARE_ID;
}
EXPORT_SYMBOL(Read_HW_ID);

static int pcb_id;
module_param(pcb_id, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(PCB_VERSION,
                 "PCB_ID judgement");

int Read_PCB_ID(void)
{
        pr_debug("PCB_ID = 0x%x \n",PCB_ID);
        pcb_id = PCB_ID;
        return PCB_ID;
}
EXPORT_SYMBOL(Read_PCB_ID);

int Read_TP_ID(void)
{
        pr_debug("TP_ID = 0x%x \n",TP_ID);
        return TP_ID;
}
EXPORT_SYMBOL(Read_TP_ID);

static int sfi_parse_oemr(struct sfi_table_header *table)
{
#ifdef CONFIG_ME560CG
        struct sfi_table_oemr *sb;
        struct sfi_oemr_table_entry *pentry;

        sb = (struct sfi_table_oemr *)table;
        pentry = (struct sfi_oemr_table_entry *)sb->pentry;
        HARDWARE_ID = pentry->hardware_id;
        PROJECT_ID = pentry->project_id;
        TP_ID = pentry->touch_id;
        pr_info("Hardware ID = %x, Project ID = %x, TP ID = %d\n", HARDWARE_ID, PROJECT_ID, TP_ID);
        switch (HARDWARE_ID)
        {
            case HW_ID_EV:
                pr_info("Hardware VERSION = EV\n");
                break;
            case HW_ID_SR1_SKU_1_2:
                pr_info("Hardware VERSION = SR1(SKU1,2)\n");
                break;
            case HW_ID_SR1_SKU_3_4:
                pr_info("Hardware VERSION = SR1(SKU3,4)\n");
                break;
            case HW_ID_PRE_ER:
                pr_info("Hardware VERSION = PRE-ER\n");
                break;
            case HW_ID_ER:
                pr_info("Hardware VERSION = ER\n");
                break;
            case HW_ID_PR:
                pr_info("Hardware VERSION = PR\n");
                break;
            case HW_ID_MP:
                pr_info("Hardware VERSION = MP\n");
                break;
            default:
                pr_info("Hardware VERSION is not defined\n");
                break;
        }
        PCB_ID = pentry->touch_id | pentry->Camera_1_2M << 2 | pentry->lcd_id << 3 | pentry->hardware_id << 4 | 
                 pentry->project_id << 7 | pentry->Camera_8M << 10 | pentry->Camera_1_2M_Lense << 11 | pentry->Camera_8M_Lense << 12 | pentry->memory << 13;

	//Thomas add for ATD Tool atd_pcb_id {
        atd_pcb_id = pentry->hardware_id << 3 | pentry->project_id;
        pr_info("ATD PCB ID=%d\n",atd_pcb_id);
        //Thomas add for ATD Tool atd_pcb_id }
#endif

#ifdef CONFIG_FE170CG
	struct sfi_table_simple *sb;
	struct sfi_oemr_table_entry *pentry;

	sb = (struct sfi_table_simple *)table;
	pentry = (struct sfi_oemr_table_entry *)sb->pentry;

	HARDWARE_ID = pentry->Hardware_ID;
	PROJECT_ID = pentry->Project_ID;
	TP_ID = pentry->TP_ID;
	RC_VERSION = pentry->IFWI_RC;
	RF_ID = pentry->RF_ID;
	PCB_ID = pentry->TP_ID | 
				 pentry->FCAM_ID << 2 | 
				 pentry->LCD_ID<< 3 | 
				 pentry->Hardware_ID << 4 |
				 pentry->DDR_ID<< 6 |
				 pentry->Project_ID<< 7 |
				 pentry->RF_ID<< 10 |
				 pentry->SIM_ID<< 11 |
				 pentry->RCAM_ID<< 12;
	printk("pentry->TP_ID=%d\n pentry->FCAM_ID=%d, pentry->LCD_ID=%d\n pentry->Hardware_ID=%d\n pentry->DDR_ID=%d\n pentry->Project_ID=%d\n pentry->RF_ID=%d\n pentry->SIM_ID=%d\n pentry->RCAM_ID=%d\n",
				 pentry->TP_ID,
		         pentry->FCAM_ID, 
				 pentry->LCD_ID, 
				 pentry->Hardware_ID,
				 pentry->DDR_ID,
				 pentry->Project_ID,
				 pentry->RF_ID,
				 pentry->SIM_ID,
				 pentry->RCAM_ID);
#endif

#ifdef CONFIG_TF103CG
	printk("TF103CG PCB ID\n");
	struct sfi_table_simple *sb;
	struct sfi_oemr_table_entry *pentry;

	sb = (struct sfi_table_simple *)table;
	pentry = (struct sfi_oemr_table_entry *)sb->pentry;

	HARDWARE_ID = pentry->Hardware_ID;
	PROJECT_ID = pentry->Project_ID;
	TP_ID = pentry->TP_ID;
	RC_VERSION = pentry->IFWI_RC;

	PCB_ID = pentry->TP_ID |
				 pentry->FCAM_ID << 2 |
				 pentry->LCD_ID<< 3 |
				 pentry->Hardware_ID << 4 |
				 pentry->Project_ID<< 6 |
				 pentry->RF_ID<< 9 |
				 pentry->RCAM_ID<< 10 |
				 pentry->EMMC_ID<< 11 |
				 pentry->DDR_ID<< 13 ;

	printk("pentry->TP_ID=%d\n pentry->FCAM_ID=%d\n pentry->LCD_ID=%d\n pentry->Hardware_ID=%d\n pentry->Project_ID=%d\n pentry->RF_ID=%d\n pentry->RCAM_ID=%d\n pentry->EMMC_ID=%d\n pentry->DDR_ID=%d\n",
				 pentry->TP_ID,
		         pentry->FCAM_ID,
				 pentry->LCD_ID,
				 pentry->Hardware_ID,
				 pentry->Project_ID,
				 pentry->RF_ID,
				 pentry->RCAM_ID,
				 pentry->EMMC_ID,
				 pentry->DDR_ID);

	//Thomas add for ATD Tool atd_pcb_id {
    atd_pcb_id = pentry->Hardware_ID << 3 | pentry->Project_ID;
    printk("ATD PCB ID = hardware_id(2bits) + project_id(3bits) = %d\n",atd_pcb_id);
    //Thomas add for ATD Tool atd_pcb_id }
#endif

	return 0;

}

static void handle_pcb_id()
{
    sfi_table_parse(SFI_SIG_OEMR, NULL, NULL, sfi_parse_oemr);
    Read_HW_ID();
    Read_PROJ_ID();
    Read_PCB_ID();
    Read_RC_VERSION();
#ifdef CONFIG_FE170CG
	Read_RF_ID();
#endif

    printk("CT add : Hardware ID = %x, Project ID = %x, TP ID = %d, PCB_ID = %x, RC_VERSION = %x\n", HARDWARE_ID, PROJECT_ID, TP_ID, PCB_ID,RC_VERSION);

}

static int pcbid_driver_probe(struct platform_device *pdev)
{        
    printk("pcbid_driver_probe+\n");

    // clovertrail+
    handle_pcb_id();
    // clovertrail-

    printk("pcbid_driver_probe-\n");

    return 0;
}

static int pcbid_driver_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver asustek_pcbid_driver __refdata = {
	.probe = pcbid_driver_probe,
	.remove = pcbid_driver_remove,
	.driver = {
		.name = "asustek_pcbid",
		.owner = THIS_MODULE,
	},
};

static int __init asustek_pcbid_init(void)
{	printk("asustek_pcbid_init+ do\n");
	return platform_driver_register(&asustek_pcbid_driver);;
}

postcore_initcall(asustek_pcbid_init);

MODULE_DESCRIPTION("ASUSTek PCBID driver");
MODULE_AUTHOR("CT Ling <ct_ling@asus.com>");
MODULE_LICENSE("GPL");


