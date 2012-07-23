// BiclopsTest.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "BiclopsAccess.h"
#include "Windows.h"

int _tmain(int argc, _TCHAR* argv[])
{
	char input[256];
	bool quit = false;

	BiclopsAccess *b = BiclopsAccess::instance();

	while(b->getMode() != b->INITIALIZED)
	{
		Sleep(100);
	}

	while(!quit)
	{
		printf("Enter command: ");
		gets(input);

		
		if(input[0] == 'p')
		{
			b->park();
		}
		else if(input[0] == 'h')
		{
			b->home();
		}
		else if(input[0] == 'g')
		{
			double p[2];
			p[0] = 0;
			p[1] = 0;

			sscanf(input, "g %lf %lf", &(p[0]), &(p[1]));

			p[0] = (p[0] / 360.0) * (2.0*3.14);
			p[1] = (p[1] / 360.0) * (2.0*3.14);

			b->setJointPosition(p);
		}
		else if(input[0] == '?')
		{
			double t1 = b->getJointPosition(0);
			double t2 = b->getJointPosition(1);

			double c1 = b->getJointDesiredPosition(0);
			double c2 = b->getJointDesiredPosition(1);

			char md[100] = "";

			BiclopsAccess::Mode mode = b->getMode();

			if(mode == BiclopsAccess::OFFLINE)
			{
				strcpy(md, "OFFLINE");
			}
			else if(mode == BiclopsAccess::INITIALIZED)
			{
				strcpy(md, "INITIALIZED");
			}
			else if(mode == BiclopsAccess::HOMING)
			{
				strcpy(md, "HOMING");
			}
			else if(mode == BiclopsAccess::POSITION)
			{
				strcpy(md, "POSITION");
			}
			else if(mode == BiclopsAccess::PARKED)
			{
				strcpy(md, "PARKED");
			}

			int deg1 = (int)(360.0 * (t1 / (2 * 3.14)));
			int deg2 = (int)(360.0 * (t2 / (2 * 3.14)));
			int com1 = (int)(360.0 * (c1 / (2 * 3.14)));
			int com2 = (int)(360.0 * (c2 / (2 * 3.14)));

			printf("[%d %d : %d %d], %s\n", deg1, deg2, com1, com2, md);
		}
		else if(input[0] == 'q')
		{
			quit = true;
		}
		else if(input[0] == '!')
		{
			printf("Possible instructions: \n");
			printf("h \t\t Home\n");
			printf("p \t\t Park\n");
			printf("g t1 t2 \t\t Move joints to t1 and t2 (in degrees)\n");
			printf("? \t\t Query State\n");
			printf("! \t\t Help\n");
		}
		else if(input[0] == 0)
		{
			// Ignore blank lines.
		}
		else
		{
			printf("Unrecognized. Enter ! for help.\n");
		}
	}

	return 0;
}

