// This brings in the iostream library 
#include <iostream>
#include <string>
#include "cylinder.h"
#include "cmath"


// data types
/*(
    short
    long
    int
    short int
    signed short
    signed short int
    unsigned short int
    float
    double
    bool
    auto 
)*/



consteval int get_value(){
    return 3;
}




const int Pen{10};
const int Marker{20};
const int Eraser{30};
const int Rectangle{40};
const int Ellipse{60};


/*
    multi-line comment
*/

// types of errors
// complier time error
// run time error
// warnings

// statement

// run line by line from the first line to the bottom
int main(int argc, char **argv){
    constexpr int value = get_value();


    // std::cout << "please input your name" << std::endl;
    // std::string name;
    //     we can use cin to get input from consel
    // std::cin >> name;
    // std::cout << "Hello " << name << std::endl;


    // endl will create another line, otherwise the next output will be in the same line of the eefirst
    std::cout << "value : " << value << std::endl;
    std::cout << "Number1" << std::endl;


    // signed values:
    int value1 {10};
    int value2 {-300};
    unsigned int value3{10};
    
    std::cout << value1 << std::endl;
    std::cout << "sizeof(value1)" << sizeof(value1) << std::endl;

    std::cout << value2 << std::endl;
    std::cout << "sizeof(value2)" << sizeof(value2) << std::endl;
    
    std::cout << value3 << std::endl;
    std::cout << "sizeof(value3)" << sizeof(value3) << std::endl;
    
    
    std::cout << "---------------------" << std::endl;
    
    auto x = 4.00;
    /*
        一、幂计算
            开方：double sqrt(double x);
            x的y次方：double pow(double x, double y);

        二、绝对值
            整型绝对值：int abs(x);
            长整型绝对值：long int abs(long int x);
            双精度绝对值：double fabs(double x);

        三、取整运算
            向上取整：double ceil(double x);
            向下取整：double floor(double x);

            2. 三角函数
            double acos(double x) 返回x的反余弦arccos(x)值,x为弧度
            double asin(double x) 返回x的反正弦arcsin(x)值,x为弧度
            double atan(double x) 返回x的反正切arctan(x)值,x为弧度
            double atan2(double x，double y) 带两个参数的反正切函数
            double cos(double x) 返回x的余弦cos(x)值,x为弧度
            double sin(double x) 返回x的正弦sin(x)值,x为弧度
            double tan(double x) 返回x的正切tan(x)值,x为弧度
            3. 幂函数
            double fmod (double x,double y); 返回两参数相除x/y的余数
            double sqrt (double x) 返回x的开平方
            double cbrt(double x) 计算x的立方根
            4. 对数函数
            double log(double x) 返回logex的值
            double log10(double x) 返回log10x的值
            double log2(double x) x的二进制对数
            5. 指数函数
            double exp(double x) 返回指数函数e^x的值
            double exp2(double x) 返回2的x次方
            double pow(double x,double y) 返回x^y的值
            double pow10(int p) 返回10^p的值
            frexp(param，n) 二进制浮点数表示方法 x=param*2^n
            double ldexp(double x,int exp);这个函数刚好跟上面那个frexp函数功能相反，它的返回值是x*2^exp
            6. 返回小数
            double modf(double value,double *iptr);拆分value值，返回它的小数部分，iptr指向整数部分（可返回）。
            double frexp(double value,int * exp);这是一个将value值拆分成小数部分f和（以2为底的）指数部分exp，并返回小数部分f，即f* 2^exp。其中f取值在0.5~1.0范围或者0
            7. 取整
            double ceil (double x); 取上整，返回比x大的最小整数

            double floor (double x); 取下整，返回比x小的最大整数，即高斯函数[x]
            double round(double x) 返回x的四舍五入值
            8. 最值
            double fmax(double x,double y) 两个参数中的最大值
            double fmin(x, y) 两个参数中的最小值
    */

    std::cout <<"double sqrt: " << sqrt(x) << std::endl;
    std::cout <<"power^2 : " << pow(x,2) << std::endl;
    std::cout <<"abs : " << abs(x) << std::endl;
    std::cout <<"ceil " << ceil(x+0.01) << std::endl;
    std::cout <<"floor " << floor(x+0.01) << std::endl;


    std::cout << "---------------------" << std::endl;

    // Operation on Data

    bool result = (1 < 2);
    if(result == true){
        std::cout << "the result is true" << std::endl;
    }



    int tool {Eraser};
    switch (tool)
    {
    case Pen :{
        std::cout << "activate Pen" << std::endl;
    }
        break;

    case Eraser :{
        std::cout << "activate Eraser" << std::endl;
    }
        break;

    case Rectangle :{
        std::cout << "activate Rectangle" << std::endl;
    }
        break;

    case Ellipse :{
        std::cout << "activate Ellipse" << std::endl;
    }
        break;

    default:{
        std::cout << "Nothing was found" << std::endl;
    }
        break;
    }

    std::cout << "---------------------" << std::endl;

    int a = 100;
    int b = 200;
    int max;

    max = (a>b)? a : b;
    auto max1 = (a>b)? a : 22.5f;
    std::cout << max << std::endl; 
    std::cout << max1 << std::endl; 

    std::cout << "---------------------" << std::endl;

    for(unsigned int i{}; i < 10; i++){
        std::cout << "I love cpp" << std::endl; 
    }

    std::cout << "---------------------" << std::endl;

    // Arrays
    int scores[] {1,2,3,4,5,6,7,8};
    
    // int scores[10];
    
    std::cout << scores[0] << std::endl; 
    std::cout << "size of array: " << std::size(scores) << std::endl;

    char message [] {"hello"};
    std::cout << "char array: " << message << std::endl;
    std::cout << "char array [0]: " << message[0] << std::endl;
     
    std::cout << "---------------------" << std::endl;
    std::cout << "start with pointers" << std::endl;
    
    // Pointers
    // Grab the adress (pointer is a special data type that store the adress)
    
    // Explicitely initialize to nullptr
    int* p_number {nullptr};
    // Initialize a pointer
    int* p_number1 {&value1};
    std::cout << p_number << std::endl;
    std::cout << p_number1 << std::endl;

    std::cout << "---------------------" << std::endl;
    std::cout << PI << std::endl;
    Cylinder clinder1(10,10);
    std::cout << "volume: " << clinder1.volume() << std::endl;

    // use pointer to search instances

    Cylinder cylinder1(10,10);
    cylinder1.volume();

    // manage a stack object through pointers
    Cylinder* p_cylinder1 = &cylinder1;
    std::cout << "volume: " << (*p_cylinder1).volume() << std::endl;
    std::cout << "volume: " << p_cylinder1->volume() << std::endl;
    
    
    Cylinder* c2 = new Cylinder(11,20);
    // deference a pointer
    std::cout << "volume c2: " << (*c2).volume() << std::endl;
    // pointer access notation "->"
    std::cout << "volume c2: " << c2->volume() << std::endl;
    delete c2; // reduce the memory from heap
    



    return 0; // this can tell the OS if the main function works or there are some problems. (finished successfully)
}