#include <iostream>
#include <cstdio>

#include "Common/CommonDefs.h"
#include "Infrastructure/Intersection.h"
#include "Vehicles/Car.h"

using namespace std;

FILE* SWERRERRORS;

int main()
{
    freopen_s(&SWERRERRORS, "Output/SwerrList.txt", "w", stderr); //changes the stderr stream output location to SwerrList.txt
    fprintf(SWERRERRORS, "SWERR Occurances:\n"); //Adds an opening line to the swerrlist

    Intersection my_intersection;
    cout << my_intersection.getRoad(SOUTH)->getLane(5)->endingPosition()[y] << endl;

    Car car(0, STRAIGHT, my_intersection.getRoad(NORTH)->getLane(5), NORMAL);

    cout << (int)car.currentState() << endl;

    while(car.currentPosition()[y] < my_intersection.getRoad(SOUTH)->getLane(car.laneNumber())->endingPosition()[y])
    {
        cout << car.currentPosition()[x] << " " << car.currentPosition()[y] << endl;
        car.drive();
    }
    cout << car.currentPosition()[x] << " " << car.currentPosition()[y] << endl;
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