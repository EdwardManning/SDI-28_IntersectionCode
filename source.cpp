#include <iostream>
#include <cstdio>

#include "Common/CommonDefs.h"

using namespace std;

FILE* SWERRERRORS;

int main()
{
    freopen_s(&SWERRERRORS, "Output/SwerrList.txt", "w", stderr);
    fprintf(SWERRERRORS, "SWERR Occurances:\n");
    return 0;
}