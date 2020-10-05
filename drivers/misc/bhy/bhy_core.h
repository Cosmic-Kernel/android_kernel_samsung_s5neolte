/*!
* @section LICENSE
 * (C) Copyright 2011~2015 Bosch Sensortec GmbH All Rights Reserved
 *
 * This software program is licensed subject to the GNU General
 * Public License (GPL).Version 2,June 1991,
 * available at http://www.fsf.org/copyleft/gpl.html
*
* @filename bhy_core.h
* @date     "Fri Apr 24 15:53:13 2015 +0800"
* @id       "109fd31"
*
* @brief
* The header file for BHy driver core
*/

#ifndef BHY_CORE_H

#include <linux/types.h>
#include <linux/wakelock.h>

#ifndef MODULE_TAG
#define MODULE_TAG "BHY"
#endif

#define BHY_DEBUG
/*
 * BHY_RESERVE_FOR_LATER_USE:
 * We need this funtion for future use
 */

#define SENSOR_NAME					"bhy"
#define SENSOR_INPUT_DEV_NAME		SENSOR_NAME
#define SENSOR_AR_INPUT_DEV_NAME	"bhy_ar"

struct bhy_data_bus {
	struct device *dev;
	s32(*read)(struct device *dev, u8 reg, u8 *data, u16 len);
	s32(*write)(struct device *dev, u8 reg, u8 *data, u16 len);
	int irq;
	int bus_type;
};

struct __attribute__((__packed__)) fifo_frame {
	u16 handle;
	u8 data[20];
};

#define BHY_FRAME_SIZE		7000
#define BHY_FRAME_SIZE_AR	50

struct frame_queue {
	struct fifo_frame *frames;
	int head;
	int tail;
	struct mutex lock;
};

struct bhy_client_data {
	struct mutex mutex_bus_op;
	struct bhy_data_bus data_bus;
	struct work_struct irq_work;
	struct input_dev *input;
	struct input_dev *input_ar;
	struct attribute_group *input_attribute_group;
	struct attribute_group *input_ar_attribute_group;
	atomic_t reset_flag;
	int sensor_sel;
	s64 timestamp_irq;
	atomic_t in_suspend;
	struct wake_lock wlock;
	u8 *fifo_buf;
	struct frame_queue data_queue;
	struct frame_queue data_queue_ar;
	u8 bmi160_foc_conf;
#ifdef BHY_DEBUG
	int reg_sel;
	int reg_len;
	int page_sel;
	int param_sel;
	int enable_irq_log;
	int enable_fifo_log;
	int hw_slave_addr;
	int hw_reg_sel;
	int hw_reg_len;
#endif /*~ BHY_DEBUG */
};


int bhy_suspend(struct device *dev);
int bhy_resume(struct device *dev);
int bhy_probe(struct bhy_data_bus *data_bus);
int bhy_remove(struct device *dev);

#ifdef CONFIG_PM
int bhy_suspend(struct device *dev);
int bhy_resume(struct device *dev);
#endif

#endif /** BHY_CORE_H */
