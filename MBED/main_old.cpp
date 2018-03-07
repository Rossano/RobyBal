/**
 ******************************************************************************
 * @file    main.cpp
 * @author  CLab
 * @version V1.0.0
 * @date    2-December-2016
 * @brief   Simple Example application for using the X_NUCLEO_IKS01A2 
 *          MEMS Inertial & Environmental Sensor Nucleo expansion board.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
*/ 

/* Includes */
#include "mbed.h"
#include "XNucleoIKS01A2.h"

/* Instantiate the expansion board */
static XNucleoIKS01A2 *mems_expansion_board = XNucleoIKS01A2::instance(D14, D15, D4, D5);

/* Retrieve the composing elements of the expansion board */
static LSM6DSLSensor *acc_gyro = mems_expansion_board->acc_gyro;

InterruptIn mybutton(USER_BUTTON);

volatile int mems_event = 0;
volatile int toggle_hw_event_enable = 0;
static int hw_event_is_enabled = 1;
uint16_t step_count = 0;

/* User button callback. */
void pressed_cb() {
  toggle_hw_event_enable = 1;
}

/* Interrupt 1 callback. */
void int1_cb() {
  mems_event = 1;
}

/* Interrupt 2 callback. */
void int2_cb() {
  mems_event = 1;
}

/* Print the orientation. */
void send_orientation() {
  uint8_t xl = 0;
  uint8_t xh = 0;
  uint8_t yl = 0;
  uint8_t yh = 0;
  uint8_t zl = 0;
  uint8_t zh = 0;
  
  acc_gyro->get_6d_orientation_xl(&xl);
  acc_gyro->get_6d_orientation_xh(&xh);
  acc_gyro->get_6d_orientation_yl(&yl);
  acc_gyro->get_6d_orientation_yh(&yh);
  acc_gyro->get_6d_orientation_zl(&zl);
  acc_gyro->get_6d_orientation_zh(&zh);
  
  if ( xl == 0 && yl == 0 && zl == 0 && xh == 0 && yh == 1 && zh == 0 ) {
    printf( "\r\n  ________________  " \
            "\r\n |                | " \
            "\r\n |  *             | " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |________________| \r\n" );
  }
  
  else if ( xl == 1 && yl == 0 && zl == 0 && xh == 0 && yh == 0 && zh == 0 ) {
    printf( "\r\n  ________________  " \
            "\r\n |                | " \
            "\r\n |             *  | " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |________________| \r\n" );
  }
  
  else if ( xl == 0 && yl == 0 && zl == 0 && xh == 1 && yh == 0 && zh == 0 ) {
    printf( "\r\n  ________________  " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |  *             | " \
            "\r\n |________________| \r\n" );
  }
  
  else if ( xl == 0 && yl == 1 && zl == 0 && xh == 0 && yh == 0 && zh == 0 ) {
    printf( "\r\n  ________________  " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |             *  | " \
            "\r\n |________________| \r\n" );
  }
  
  else if ( xl == 0 && yl == 0 && zl == 0 && xh == 0 && yh == 0 && zh == 1 ) {
    printf( "\r\n  __*_____________  " \
            "\r\n |________________| \r\n" );
  }
  
  else if ( xl == 0 && yl == 0 && zl == 1 && xh == 0 && yh == 0 && zh == 0 ) {
    printf( "\r\n  ________________  " \
            "\r\n |________________| " \
            "\r\n    *               \r\n" );
  }
  
  else {
    printf( "None of the 6D orientation axes is set in LSM6DSL - accelerometer.\r\n" );
  }
}

/* Simple main function */
int main() {
  /* Attach callback to User button press */
  mybutton.fall(&pressed_cb);
  /* Attach callback to LSM6DSL INT1 */
  acc_gyro->attach_int1_irq(&int1_cb);
  /* Attach callback to LSM6DSL INT2 */
  acc_gyro->attach_int2_irq(&int2_cb);
  
  /* Enable LSM6DSL accelerometer */
  acc_gyro->enable_x();
  /* Enable HW events. */
  acc_gyro->enable_pedometer();
  acc_gyro->enable_tilt_detection();
  acc_gyro->enable_free_fall_detection();
  acc_gyro->enable_single_tap_detection();
  acc_gyro->enable_double_tap_detection();
  acc_gyro->enable_6d_orientation();
  acc_gyro->enable_wake_up_detection();
  
  printf("\r\n--- Starting new run ---\r\n");
 
  while(1) {
    if (mems_event) {
      mems_event = 0;
      LSM6DSL_Event_Status_t status;
      acc_gyro->get_event_status(&status);
      if (status.StepStatus) {
        /* New step detected, so print the step counter */
        acc_gyro->get_step_counter(&step_count);
        printf("Step counter: %d\r\n", step_count);
      }

      if (status.FreeFallStatus) {
        /* Output data. */
        printf("Free Fall Detected!\r\n");
      }

      if (status.TapStatus) {
        /* Output data. */
        printf("Single Tap Detected!\r\n");
      }

      if (status.DoubleTapStatus) {
        /* Output data. */
        printf("Double Tap Detected!\r\n");
      }

      if (status.D6DOrientationStatus) {
        /* Send 6D Orientation */
        send_orientation();
      }

      if (status.TiltStatus) {
        /* Output data. */
        printf("Tilt Detected!\r\n");
      }

      if (status.WakeUpStatus) {
        /* Output data. */
        printf("Wake Up Detected!\r\n");
      }
    }

    if (toggle_hw_event_enable) {
      toggle_hw_event_enable = 0;
      if (hw_event_is_enabled == 0) {
        /* Enable HW events. */
        acc_gyro->enable_pedometer();
        acc_gyro->enable_tilt_detection();
        acc_gyro->enable_free_fall_detection();
        acc_gyro->enable_single_tap_detection();
        acc_gyro->enable_double_tap_detection();
        acc_gyro->enable_6d_orientation();
        acc_gyro->enable_wake_up_detection();
        hw_event_is_enabled = 1;
      } else {
        acc_gyro->disable_pedometer();
        acc_gyro->disable_tilt_detection();
        acc_gyro->disable_free_fall_detection();
        acc_gyro->disable_single_tap_detection();
        acc_gyro->disable_double_tap_detection();
        acc_gyro->disable_6d_orientation();
        acc_gyro->disable_wake_up_detection();
        hw_event_is_enabled = 0;
      }
    }
  }
}
