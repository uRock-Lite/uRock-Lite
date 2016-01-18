Description
---------
- A simple example for ADC to DAC by DMA.
  - ADC : PC3
  - DAC : PA5


Prerequisites
-------------
- Hardware
  - [STM32F429ZIT6](http://wiki.csie.ncku.edu.tw/embedded/STM32F429)
  - [Pre - Amp Circuit]()
  - [Pose- Low pass filiter]()
  - Electric guitar
  - AMP
- Software
  - [CooCox](http://www.coocox.org/)
    - Build project, flash and debug.
  - [Cubemx](http://www.st.com/web/catalog/tools/FM147/CL1794/SC961/SS1533/PF259242?sc=stm32cube)
    - A GUI for hardware configuration and code generator.
  - [GNU Toolchain for ARM](https://launchpad.net/gcc-arm-embedded)


Steps
---------
- 1. Connect PC3 with Pre-Amp Circuit and and your guitar. 
- 2. Connect PA5 with Pose-Amp Circuit and and your AMP.
- 3. Open the project by coocox IDE.
- 4. Plug-in stm32f429i.
- 5. Click 'debug' button and good to go.
  - You should listen your guitar bypass from stm32f429i to AMP. 


Note
---------
- You should put your code in the block like below. Cubemx will remain your code after generating new code.
``` 
	/* USER CODE END N */ 
	/* USER CODE END N */ 
```


Reference
---------
* [uRock](https://github.com/sonicyang/uRock)