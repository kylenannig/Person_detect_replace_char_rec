// Copyright (c) 2021, XMOS Ltd, All rights reserved

#include <platform.h>
#include <xs1.h>

#include "FreeRTOS.h"

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "app_conf.h"
#include "model_runner.h"
#include "person_detect_task.h"
#include "person_detect_model_data.h"
#include "person_detect_model_runner.h"

#ifdef OUTPUT_IMAGE_STREAM
#include "xscope.h"
#endif

#define IMAGE_SIZE (96 * 96)

typedef struct model_runner_args {
  QueueHandle_t input_queue;
  rtos_intertile_address_t *intertile_addr;
} model_runner_args_t;

typedef struct app_task_args {
  QueueHandle_t input_queue;
  rtos_intertile_address_t *intertile_addr;
  rtos_gpio_t *gpio_ctx;
} app_task_args_t;

#define TENSOR_ARENA_SIZE (250000)

static void person_detect_app_task(void *args) {
  app_task_args_t *targs = (app_task_args_t *)args;
  QueueHandle_t input_queue = targs->input_queue;
  rtos_intertile_address_t *adr = targs->intertile_addr;
  rtos_gpio_t *gpio_ctx = targs->gpio_ctx;
  uint8_t *img_buf = NULL;
  uint8_t *output_tensor;
  int output_tensor_len;
  uint8_t ai_img_buf[IMAGE_SIZE];


  rtos_gpio_port_id_t led_port = 0;
  uint32_t val = 0;
  int toggle = 0;

  led_port = rtos_gpio_port(PORT_LEDS);
  rtos_gpio_port_enable(gpio_ctx, led_port);
  rtos_gpio_port_out(gpio_ctx, led_port, val);

  while (1) {
    rtos_printf("Wait for next image...\n");
    xQueueReceive(input_queue, &img_buf, portMAX_DELAY);

    /* img_buf[i%2] contains the values we want to pass to the ai task */
    for (int i = 0; i < (IMAGE_SIZE * 2); i++) {
      if ((i % 2)) {
        ai_img_buf[i >> 1] = img_buf[i] ;                 //replace with thresholding statement               
        if (ai_img_buf[i >> 1] > 0x7F) {
          ai_img_buf[i >> 1] = 0xFF;
        }
        else {
          ai_img_buf[i >> 1] = 0x00;
        }   
        //rtos_printf("ai_img_buf_value %d \n", ai_img_buf[i >> 1]);  
                                                   //threshold 127 that will alwasy return 00 or ff
        
      }
    }
    vPortFree(img_buf);


#ifdef OUTPUT_IMAGE_STREAM
    taskENTER_CRITICAL();
    {
      xscope_bytes(INPUT_IMAGE, IMAGE_SIZE, (const unsigned char *)ai_img_buf);
    }
    taskEXIT_CRITICAL();
#endif

    rtos_intertile_tx(adr->intertile_ctx, adr->port, ai_img_buf, IMAGE_SIZE);
    output_tensor_len = rtos_intertile_rx(
        adr->intertile_ctx, adr->port, (void **)&output_tensor, portMAX_DELAY);
    rtos_printf("\noutput_tensor[0](0) %d\noutput_tensor[1] (1) %d\noutput_tensor[2] (2) %d\nnoutput_tensor[3] (3) %d\nnoutput_tensor[4] (4) %d\nnoutput_tensor[5] (5) %d\nnoutput_tensor[6] (6) %d\nnoutput_tensor[7] (7) %d\nnoutput_tensor[8] (8)%d\nnoutput_tensor[9] (9) %d\nnoutput_tensor[10] (A) %d\nnoutput_tensor[11] (B) %d\nnoutput_tensor[12] (C) %d\nnoutput_tensor[13] (D) %d\nnoutput_tensor[14] (E) %d\nnoutput_tensor[15] (F) %d\nnoutput_tensor[16] (G) %d\nnoutput_tensor[17] (H) %d\nnoutput_tensor[18] (I) %d\nnoutput_tensor[19] (J) %d\nnoutput_tensor[20] (K) %d\noutput_tensor[21] (L) %d\noutput_tensor[22] (M) %d\noutput_tensor[23] (N) %d\noutput_tensor[24] (o) %d\noutput_tensor[25] (P) %d\noutput_tensor[26] (Q) %d\noutput_tensor[27] (R)%d\noutput_tensor[28] (S) %d\noutput_tensor[29] (T) %d\noutput_tensor[30] (U) %d\noutput_tensor[31] (V) %d\noutput_tensor[32] (W) %d\noutput_tensor[33] (X) %d\noutput_tensor[34] (Y) %d\noutput_tensor[35] (Z)%d\noutput_tensor[36]%d\noutput_tensor[37]%d\noutput_tensor[38]%d\n", output_tensor[0],
                output_tensor[1], output_tensor[2], output_tensor[3], output_tensor[4], output_tensor[5], output_tensor[6], output_tensor[7], output_tensor[8], output_tensor[9], output_tensor[10], output_tensor[11], output_tensor[12], output_tensor[13], output_tensor[14], output_tensor[15], output_tensor[16], output_tensor[17], output_tensor[18], output_tensor[19], output_tensor[20], output_tensor[21], output_tensor[22], output_tensor[23], output_tensor[24], output_tensor[25], output_tensor[26], output_tensor[27], output_tensor[28], output_tensor[29], output_tensor[30], output_tensor[31], output_tensor[32], output_tensor[33], output_tensor[34], output_tensor[35], output_tensor[36], output_tensor[37],output_tensor[38]);

#ifdef OUTPUT_IMAGE_STREAM
    taskENTER_CRITICAL(); 
    { 
      xscope_bytes(OUTPUT_TENSOR, output_tensor_len,
                   (const unsigned char *)output_tensor);
    }
    taskEXIT_CRITICAL();
#endif

    val = ((toggle++) & 0x01) << 3;
    val |= (output_tensor[0] > output_tensor[1]);
    rtos_gpio_port_out(gpio_ctx, led_port, val);

    vPortFree(output_tensor);
  }
}

static void person_detect_runner_rx(void *args) {
  model_runner_args_t *targs = (model_runner_args_t *)args;
  QueueHandle_t q = targs->input_queue;
  rtos_intertile_address_t *adr = targs->intertile_addr;
  uint8_t *input_tensor;
  int input_tensor_len;


  //Kyle: defining row and column counting variables for cropping
  int column = 0;
  int row = 0;

  //test variables
  //int temp = 0x00;
  int test = 0;

  //kyle definining new arrays
  //uint8_t *feed_to_ai[80*80*3];
  uint8_t crop_buf[80*80];

  while (1) {
    input_tensor_len = rtos_intertile_rx(adr->intertile_ctx, adr->port,
                                         (void **)&input_tensor, portMAX_DELAY);
    //TODO: crop 96x96x1 to 80x80x1
    //Loop through every element, have counter counting rows and columns (calling them rows and columns 0 through 95)
    //k represents the row counter, j represents the column counter
    //if in first 80x80 gets included in crop_buf

    for (int i = 0; i < (IMAGE_SIZE); i++) {
      if (column == 96){
      column=0;
      row++;}
      if(column<80 && row<80){
        crop_buf[i]= input_tensor[i];
        //rtos_printf("input-tensor %d\n", input_tensor[i]);
      }
      column++;
    }

   test = sizeof(crop_buf);
   rtos_printf("size of crop_buf %d \n", test);
   
   //TODO: modify contents of input tensor using tripling to expand 80x80x1 to 80x80x3

   for (int i = 0; i < (80*80); i++) {
     for (int j = 0; j < 3; j++){
       input_tensor[3*i + j] = crop_buf[i];
      }
   }


/*test
   for (int i = 0; i < (96*96+10); i++) { 
     input_tensor[i]=temp;
     
     rtos_printf("\nvalue of input_tensor[0] %d [1] %d", i);
   }
*/
    xQueueSend(q, &input_tensor, portMAX_DELAY);
  }
}



static void person_detect_task_runner(void *args) {
  model_runner_args_t *targs = (model_runner_args_t *)args;
  QueueHandle_t q = targs->input_queue;
  rtos_intertile_address_t *adr = targs->intertile_addr;
  size_t req_size = 0;
  uint8_t *interpreter_buf = NULL;
  int8_t *input_buffer = NULL;
  size_t input_size = 0;
  int8_t *output_buffer = NULL;
  size_t output_size = 0;
  model_runner_t *model_runner_ctx = NULL;
  uint8_t *tensor_arena = NULL;
  uint8_t *input_tensor;
//rtos_printf("print 0\n");
  tensor_arena = pvPortMalloc(TENSOR_ARENA_SIZE);
//rtos_printf("print 1\n");
  model_runner_init(tensor_arena, TENSOR_ARENA_SIZE);
//rtos_printf("print 2\n");
  req_size = model_runner_buffer_size_get();
//rtos_printf("print 3\n");
  interpreter_buf = pvPortMalloc(req_size);
//rtos_printf("print 4\n");
  model_runner_ctx = pvPortMalloc(sizeof(model_runner_t));
//rtos_printf("print 5\n");
  person_detect_model_runner_create(model_runner_ctx, interpreter_buf);
  if (model_runner_allocate(model_runner_ctx, person_detect_model_data) != 0) {
    rtos_printf("Invalid model provided!\n");

    vPortFree(tensor_arena);
    vPortFree(interpreter_buf);
    vPortFree(model_runner_ctx);

    vTaskDelete(NULL);
  }

  input_buffer = model_runner_input_buffer_get(model_runner_ctx);
  input_size = model_runner_input_size_get(model_runner_ctx);
  rtos_printf("image size is %d expected; %d\n", input_size, 80*80*3);
  output_buffer = model_runner_output_buffer_get(model_runner_ctx);
  output_size = model_runner_output_size_get(model_runner_ctx);

  while (1) {
    rtos_printf("Wait for input tensor...\n");
    xQueueReceive(q, &input_tensor, portMAX_DELAY);

    memcpy(input_buffer, input_tensor, input_size);
    vPortFree(input_tensor);

    rtos_printf("Running inference...\n");
    model_runner_invoke(model_runner_ctx);
    //model_runner_profiler_summary_print(model_runner_ctx);
    



    rtos_intertile_tx(adr->intertile_ctx, adr->port, output_buffer,
                      output_size);
  }
}

void person_detect_app_task_create(rtos_intertile_address_t *intertile_addr,
                                   rtos_gpio_t *gpio_ctx, unsigned priority,
                                   QueueHandle_t input_queue) {
  if (gpio_ctx != NULL) {
    app_task_args_t *args = pvPortMalloc(sizeof(app_task_args_t));

    configASSERT(args);

    args->input_queue = input_queue;
    args->intertile_addr = intertile_addr;
    args->gpio_ctx = gpio_ctx;

    xTaskCreate((TaskFunction_t)person_detect_app_task, "person_detect_app",
                RTOS_THREAD_STACK_SIZE(person_detect_app_task), args, priority,
                NULL);
  } else {
    rtos_printf("Invalid gpio ctx provided\n");
  }
}

void person_detect_model_runner_task_create(
    rtos_intertile_address_t *intertile_addr, unsigned priority) {
  model_runner_args_t *args = pvPortMalloc(sizeof(model_runner_args_t));
  QueueHandle_t input_queue = xQueueCreate(1, sizeof(int32_t *));

  configASSERT(args);
  configASSERT(input_queue);

  args->input_queue = input_queue;
  args->intertile_addr = intertile_addr;

  xTaskCreate((TaskFunction_t)person_detect_task_runner, "person_detect", 500,
              args, priority, NULL);

  xTaskCreate((TaskFunction_t)person_detect_runner_rx, "person_detect_rx",
              RTOS_THREAD_STACK_SIZE(person_detect_runner_rx), args,
              priority - 1, NULL);
}
