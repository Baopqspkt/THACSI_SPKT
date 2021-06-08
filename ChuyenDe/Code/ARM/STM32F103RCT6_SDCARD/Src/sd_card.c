#include "sd_card.h"

FATFS fs;  // file system
FIL fil; // File
FILINFO fno;
FRESULT fresult;  // result
UINT br, bw;  // File read/write count

/**** capacity related *****/
FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;

uint8_t sd_card_check_available(void)
{
    fresult = f_mount(&fs, "/", 1);
  	if (fresult != FR_OK) 
    {
        printf ("ERROR!!! in mounting SD CARD...\n\r");
        return 1;
    }
  	else 
    {
        printf("SD CARD mounted successfully...\n\r");
    }
    
    
    f_getfree("", &fre_clust, &pfs);

  	total = (uint32_t)((pfs->n_fatent - 2) * pfs->csize * 0.5);
    printf( "SD CARD Total Size: \t%d\n\r",total);
  	free_space = (uint32_t)(fre_clust * pfs->csize * 0.5);
    printf( "SD CARD Free Space: \t%d\n\r",free_space);
    return 0;
}

void sd_card_write(void)
{
    if(!sd_card_check_available())
    {
        fresult = f_open(&fil, "file1.txt", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
  	    /* Writing text */
  	    f_puts("This data is from the FILE1.txt. And it was written using ...f_puts... ", &fil);
  	    /* Close file */
  	    fresult = f_close(&fil);
  	    if (fresult == FR_OK)
            printf("File1.txt created and the data is written \n\r");
    }
}
