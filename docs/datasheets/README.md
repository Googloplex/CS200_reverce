# Chip and board documentation (not included in the repository — third-party PDFs)

- Linsynix LCP037BT32EU8 (MCU + predriver in one package), datasheet: https://datasheet.lcsc.com/datasheet/pdf/cd99eebd396de353c9435afa827be477.pdf?productCode=C5140366
- LCM32F037 core (Cortex-M0, 72 MHz, 32 KB Flash, 4 KB SRAM), datasheet: https://wmsc.lcsc.com/wmsc/upload/file/pdf/v2/lcsc/2208260930_LNCHIP-LCM32F037H6S8_C5140364.pdf
- Reference Manual V7.11, SDK V1.2.0, Keil pack LCM32F0xx_DFP 0.4.72 (SVD, Flash algorithm): https://www.linsynix.com/product-dmcu/10.html?proid=2
- NCEP6090AGU MOSFETs (60 V / 90 A): https://jlcpcb.com/partdetail/515514-NCEP6090AGU/C502993
- Kpower HB200T: https://www.kpower.com/hb200t_servo

RM text with Chinese characters for grep is produced by `tools/rm_pages.py --extract` from the PDF placed here as `LCM32F037_RM_V7.11.pdf`.
