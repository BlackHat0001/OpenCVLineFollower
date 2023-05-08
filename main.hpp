#ifndef MAIN_HPP_INCLUDED
#define MAIN_HPP_INCLUDED

// Function declarations
void setup(void);                   //Function to configure the GPIO and devices for operation
int main(int argc, char** argv);
void GetLinePosition(int hsvArr[]);
int LookForSymbol(int hsvArr[]);
float MapFunction(float x, float inMin, float inMax, float outMin, float outMax);
double angle(Point pt1, Point pt2, Point pt0);


#endif // MAIN_HPP_INCLUDED
