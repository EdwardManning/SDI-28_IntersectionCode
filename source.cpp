#include <iostream>
#include <cstdio>

#include "Common/CommonDefs.h"
#include "Infrastructure/Intersection.h"

using namespace std;

FILE* SWERRERRORS;

int main()
{
    freopen_s(&SWERRERRORS, "Output/SwerrList.txt", "w", stderr); //changes the stderr stream output location to SwerrList.txt
    fprintf(SWERRERRORS, "SWERR Occurances:\n"); //Adds an opening line to the swerrlist

    Intersection my_intersection;
    cout << my_intersection.getRoad(EAST)->getLane(6)->startingPosition()[y] << endl;
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