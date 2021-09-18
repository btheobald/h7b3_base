#ifndef __RK043FN48H_H
#define __RK043FN48H_H

#ifdef __cplusplus
 extern "C" {
#endif 

#define  RK043FN48H_WIDTH   ((uint16_t)480)  // LCD PIXEL WIDTH
#define  RK043FN48H_HEIGHT  ((uint16_t)272)  // LCD PIXEL HEIGHT
  
#define  RK043FN48H_HSYNC   ((uint16_t)1)   // Horizontal synchronization
#define  RK043FN48H_HBP     ((uint16_t)8)   // Horizontal back porch
#define  RK043FN48H_HFP     ((uint16_t)2)   // Horizontal front porch
#define  RK043FN48H_VSYNC   ((uint16_t)1)   // Vertical synchronization
#define  RK043FN48H_VBP     ((uint16_t)2)    // Vertical back porch
#define  RK043FN48H_VFP     ((uint16_t)1)    // Vertical front porch

#define  RK043FN48H_AW      RK043FN48H_HSYNC+RK043FN48H_HBP+RK043FN48H_WIDTH
#define  RK043FN48H_AH      RK043FN48H_VSYNC+RK043FN48H_VBP+RK043FN48H_HEIGHT

#define  RK043FN48H_TW      RK043FN48H_AW+RK043FN48H_HFP
#define  RK043FN48H_TH      RK043FN48H_AH+RK043FN48H_VFP

#define  RK043FN48H_FREQUENCY_DIVIDER    5    // LCD Frequency divider

#ifdef __cplusplus
}
#endif

#endif // __RK043FN48H_H 