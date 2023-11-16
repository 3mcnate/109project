void lcd_init(void);

void lcd_moveto(unsigned char, unsigned char);

void lcd_stringout(char *);

void lcd_writecommand(unsigned char);

void lcd_writedata(unsigned char);

void lcd_errormsg(char* str);

void lcd_write_thresh_val(uint8_t val, char col);

void lcd_write_qmark(char col);

void lcd_write_equals(char col);