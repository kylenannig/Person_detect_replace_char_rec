################
Person Detection
################

This example demonstrates how to integrate the xCORE FreeRTOS port with the TensorFlow person detection example application.  This application places the model in LPDDR.

The example uses an `ArduCam 2MP Plus OV2640 Mini Module SPI camera <https://www.arducam.com/product/arducam-2mp-spi-camera-b0067-arduino/>`__ to capture a 96x96 8 bit grayscale image.  The application will attempt to classify whether the image contains a person or does not contain a person.

When a person is detected LED 0 turns on.  When a person is not detected LED 0 turns off.  LED 3 toggles after each inference.  Additionally, a Python 3 script is provided to output image and output tensor pairs to the host machine.  This script requires `Numpy <https://numpy.org/>`__ and `Matplotlib <https://matplotlib.org/>`__.

**************
Hardware setup
**************

Materials Required
==================

- `ArduCam 2MP Plus OV2640 Mini Module SPI camera <https://www.arducam.com/product/arducam-2mp-spi-camera-b0067-arduino/>`__
- 0.100" female to female jumper wires
- 0.100" (2.54mm) throughhole header pins
- Soldering tools and supplies

To connect the ArduCam to the Explorer Board use the schematic and following table for reference:

=======     ==============
ArduCam     Explorer Board
=======     ==============
CS          J14  : X0D00
MOSI        J6   : MOSI
MISO        J6   : MISO
SCK         J6   : CLK
GND         J5   : GND
VCC         TP8  : 3V3
SDA         TP19 : SDA_IOL
SCL         TP18 : SCL_IOL
=======     ==============

*********************
Building the firmware
*********************

Using SRAM memory
=================

Run make:

.. code-block:: console

    $ make

Using external DDR memory
=========================

If your board supports LPDDR, you may also place your neural network in the external DDR memory.  Currently, only the Explorer Board supports LPDDR.

To build with the model stored in LPDDR, replace the call to make above with the following:

.. code-block:: console

    $ make USE_EXTMEM=1

Running the firmware
====================

This demo can be run with only GPIO or GPIO and host output.

Running with GPIO only:

.. code-block:: console

    $ xrun --xscope ../bin/person_detection.xe

Running with GPIO and host:

.. code-block:: console

    $ xrun --xscope --xscope-port localhost:10234 ../bin/person_detection.xe

In a second terminal:

.. code-block:: console

    $ python image_viewer.py

Once the host script connects to the xscope server the image and associated output tensor values will be displayed.

.. image:: images/person.png
    :align: left

.. image:: images/not_person.png
    :align: left


********************
Optimizing the model
********************

If the model is retrained, you will need to optimize it for xcore.ai.  Refer to the documentation in ai_tools/doc for information on this process.

******************
Training the model
******************

You may wish to retrain this model.  This should rarely be necessary. However, if you would like to learn more about how this model is trained, see: https://github.com/tensorflow/tensorflow/blob/master/tensorflow/lite/micro/examples/person_detection/training_a_model.md
