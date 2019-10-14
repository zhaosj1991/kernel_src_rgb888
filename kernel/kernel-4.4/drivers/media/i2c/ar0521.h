#ifndef __AR0521_H__
#define __AR0521_H__

#include <linux/device.h>
#include <linux/v4l2-controls.h>

int cam_gpio_register(struct device *dev,
                      unsigned pin_num);

void cam_gpio_deregister(struct device *dev,
                         unsigned pin_num);

int cam_gpio_ctrl(struct device *dev,
                  unsigned pin_num, int ref_inc, bool active_high);


#define GALAXY_CAMERA_CID_BASE              (V4L2_CTRL_CLASS_CAMERA | 0x2100)
#define GALAXY_CAMERA_CID_CHIP_VERSION      (GALAXY_CAMERA_CID_BASE+0)
#define GALAXY_CAMERA_CID_GAIN              (GALAXY_CAMERA_CID_BASE+1)
#define GALAXY_CAMERA_CID_EXPOSURE          (GALAXY_CAMERA_CID_BASE+2)
#define GALAXY_CAMERA_CID_WIDTH             (GALAXY_CAMERA_CID_BASE+3)
#define GALAXY_CAMERA_CID_HEIGHT            (GALAXY_CAMERA_CID_BASE+4)
#define GALAXY_CAMERA_CID_OFFSET_X          (GALAXY_CAMERA_CID_BASE+5)
#define GALAXY_CAMERA_CID_OFFSET_Y          (GALAXY_CAMERA_CID_BASE+6)
#define GALAXY_CAMERA_CID_OFFSET_X_END      (GALAXY_CAMERA_CID_BASE+7)
#define GALAXY_CAMERA_CID_OFFSET_Y_END      (GALAXY_CAMERA_CID_BASE+8)
#define GALAXY_CAMERA_CID_LINE_LENGTH       (GALAXY_CAMERA_CID_BASE+9)



#define GALAXY_CAMERA_CID_CHIP_TEMPER       (GALAXY_CAMERA_CID_BASE+1000)
#define GALAXY_CAMERA_CID_FRAME_PREAMBLE    (GALAXY_CAMERA_CID_BASE+1001)
#define GALAXY_CAMERA_CID_LINE_PREAMBLE     (GALAXY_CAMERA_CID_BASE+1002)


#define AR0521_TIMING_REG20                 0x3820
#define VERTICAL_FLIP                       ((0x1 << 1) | (0x1 << 6))
#define AR0521_TIMING_REG21                 0x3821
#define HORIZONTAL_MIRROR_MASK              (0x3 << 1)


#endif
/* __CAMERA_GPIO_H__ */
