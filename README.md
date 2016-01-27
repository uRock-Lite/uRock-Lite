Description
---------
- This is a digital effect paddle controlled by android app.
  - ADC : PC3
  - DAC : PA5
  - USART for bluetooth
    - PA9 : TX (Connect with RX)
    - PA10: RX (Connect with TX)


Prerequisites
-------------
- Hardware
  - Core
    - [STM32F429ZIT6](http://wiki.csie.ncku.edu.tw/embedded/STM32F429)
    - [Pre - Amp Circuit](https://hackpad.com/Harware-for-STM32F4-Guitar-Fx-gnyygrcy2R8#:h=Pre-AMP)
    - [Pose- Low pass filiter](https://hackpad.com/Harware-for-STM32F4-Guitar-Fx-gnyygrcy2R8#:h=Low-Pass-Filter-&-)
    - Bluethooth module
      - Ex: HC-06
  - Peripheral
    - Electric guitar
    - Guitar AMP
    - Smartphoon

- Software
  - [CooCox](http://www.coocox.org/)
    - Build project, flash and debug.
  - [Cubemx](http://www.st.com/web/catalog/tools/FM147/CL1794/SC961/SS1533/PF259242?sc=stm32cube)
    - A GUI for hardware configuration and code generator.
  - [GNU Toolchain for ARM](https://launchpad.net/gcc-arm-embedded)
  - [Android controller](https://github.com/uRock-Lite/uRock-Lite_Controller_v5.0)



Steps
---------
- 1. Connect PC3 with Pre-Amp Circuit and and your guitar. 
- 2. Connect PA5 with Pose-Amp Circuit and and your AMP.
- 3. Connect bluetooth module at PA9 and PA10.
- 4. Open the project by coocox IDE.
- 5. Plug-in stm32f429i.
- 6. Click 'debug' button and then 'run'.
  - You should listen your guitar bypass from stm32f429i to AMP. 
- 7. Install android dsp controller on your smartphone.
  - - [Download](https://drive.google.com/file/d/0B-mQHjLj83knWkVpSnFrT09QTlE/view?usp=sharing)
- 8. Connect with uRock-Lite, and then good to go. 
  - Now you can change effect of every stage and parameters on app.


Note
---------
- You should put your code in the block like below. Cubemx will remain your code after generating new code.
``` 
  /* USER CODE BEGIN N */ 
  /* USER CODE END N */ 
```

Reference
---------
* [Document](https://docs.google.com/document/d/1sc4C5khjE6RsQxk8Grw73ENYcpimhQgjT88qbRWdhjI/edit)
* [WorkRecord](https://2015embedded2.hackpad.com/Work-Record-upHjz1fcWD6)
* [uRock](https://github.com/sonicyang/uRock)