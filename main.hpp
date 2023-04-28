#ifndef MAIN_HPP_INCLUDED
#define MAIN_HPP_INCLUDED

// Function declarations
void setup(void);                   //Function to configure the GPIO and devices for operation
int main(int argc, char** argv);
float GetLinePosition(int H_lower, int H_higher, int S_lower, int S_higher, int V_lower, int V_higher);
float MapFunction(float x, float inMin, float inMax, float outMin, float outMax);



#endif // MAIN_HPP_INCLUDED
