#pragma once
/*
*Convert float to string type
*Written by Pegasus Yu in 2022
*stra: string address as mychar from char mychar[];
*float: float input like 12.345
*flen: fraction length as 3 for 12.345
*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

void py_f2s4printf(char * stra, float x, uint8_t flen)
{
	uint32_t base;
	int64_t dn;
	char mc[32];

	base = pow(10,flen);
	dn = x*base;
	sprintf(stra, "%d.", (int)(dn/base));
	dn = abs(dn);
	if(dn%base==0)
	{
		for(uint8_t j=1;j<=flen;j++)
		{
			stra = strcat(stra, "0");
		}
		return;
	}
	else
	{
		if(flen==1){
			sprintf(mc, "%d", (int)(dn%base));
			stra = strcat(stra, mc);
			return;
		}

		for(uint8_t j=1;j<flen;j++)
		{
			if((dn%base)<pow(10,j))
			{
				for(uint8_t k=1;k<=(flen-j);k++)
				{
					stra = strcat(stra, "0");
				}
				sprintf(mc, "%d", (int)(dn%base));
				stra = strcat(stra, mc);
				return;
			}
		}
		sprintf(mc, "%d", (int)(dn%base));
		stra = strcat(stra, mc);
		return;
	}
}