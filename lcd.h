/*
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * <joerg@FreeBSD.ORG> wrote this file.  As long as you retain this notice you
 * can do whatever you want with this stuff. If we meet some day, and you think
 * this stuff is worth it, you can buy me a beer in return.        Joerg Wunsch
 * ----------------------------------------------------------------------------
 *
 * Stdio demo, upper layer of LCD driver.
 *
 * $Id$
 */

/*
 * Initialize LCD controller.  Performs a software reset.
 */
void	lcd_init(void);

/*
 * Send one character to the LCD.
 */
int	lcd_putchar(char c, FILE *stream);

void lcd_clear (void);

void lcd_setpos (unsigned char pos);
void lcd_clearline(unsigned char pos);

#define LCD_LINE1 0
#define LCD_LINE2 0x40
#define LCD_LINE3 0x10
#define LCD_LINE4 0x50

