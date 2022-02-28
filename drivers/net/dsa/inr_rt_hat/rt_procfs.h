/**
*@file
*@brief pric-fs definitions
*@author M.Ulbricht 2021
*@copyright GNU Public License v3.
*
**/
void PROC_FS_exit (void);
int PROC_FS_init (void);

struct mdio_data {
	uint32_t addr;
	uint32_t val;
	};
