#include <iostream>
#include <cstdio>

#include "./Simulation.h"

using namespace std;

FILE* SWERRERRORS;

int main()
{
    freopen_s(&SWERRERRORS, "Output/SwerrList.txt", "w", stderr); //changes the stderr stream output location to SwerrList.txt
    fprintf(SWERRERRORS, "SWERR Occurances:\n"); //Adds an opening line to the swerrlist

    Simulation sim;

    return 0;
}

/*
*   Name:
*
*   Description:
*
*   Input:
*
*   Output:
*
*/

/*
*   Name:
*
*   Description:
*
*   Type:
*/