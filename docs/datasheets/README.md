# Документация на чип и плату (в репозиторий не включена — чужие PDF)

- Linsynix LCP037BT32EU8 (MCU + предрайвер в одном корпусе), даташит: https://datasheet.lcsc.com/datasheet/pdf/cd99eebd396de353c9435afa827be477.pdf?productCode=C5140366
- Ядро LCM32F037 (Cortex-M0, 72 МГц, 32 КБ Flash, 4 КБ SRAM), даташит: https://wmsc.lcsc.com/wmsc/upload/file/pdf/v2/lcsc/2208260930_LNCHIP-LCM32F037H6S8_C5140364.pdf
- Reference Manual V7.11, SDK V1.2.0, Keil pack LCM32F0xx_DFP 0.4.72 (SVD, алгоритм Flash): https://www.linsynix.com/product-dmcu/10.html?proid=2
- Ключи NCEP6090AGU (60 В / 90 А): https://jlcpcb.com/partdetail/515514-NCEP6090AGU/C502993
- Kpower HB200T: https://www.kpower.com/hb200t_servo

Текст RM с иероглифами для grep делается `tools/rm_pages.py --extract` из PDF, положенного сюда как `LCM32F037_RM_V7.11.pdf`.
