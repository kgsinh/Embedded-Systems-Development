/*******************************************************************************
* File Name          : MidtermVS.cpp
* Description        : It will take three values from the user and those values
					   will be passed in function and the function will decide and
					   return the maximum value and print it and if all the values
					   are 0, the program will terminate.
*
* Author:            : Kunalsinh Gohil
* Date:              : 17/10/2018
******************************************************************************
*/

#include "stdafx.h"


// FUNCTION      : maximumNumber
// DESCRIPTION   : The function takes three parameters and decides which one is 
//                 greater and than returns the greater value
//   
// PARAMETERS    : parameters passsed are num1, num2 and num2
//
// RETURNS       : returns the maximum integer value
int maximumNumber(int num1, int num2, int num3)
{
	if (num1 > num2 && num1 > num3)
	{
		return num1;
	}
	if (num2 > num1 && num2 > num3)
	{
		return num2;
	}
	if (num3 > num1 && num3 > num2)
	{
		return num3;
	}

}

int main()
{
	while (1)
	{
		int num1, num2, num3, maxValue;
		printf("Enter three separate numbers:");
		scanf_s("%d %d %d", &num1, &num2, &num3); // takres three values from the user
		if (num1 == 0 && num2 == 0 && num3 == 0) // if all values are 0 it will terminate the code
		{
			break;
		}
		maxValue = maximumNumber(num1, num2, num3); //function call and the returned value will be stored in maxValue 
		printf("Maximum value is %d\n", maxValue); //It will print the maximum value
	}


	return 0;
}

