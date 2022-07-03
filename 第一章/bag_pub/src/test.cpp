#include <iostream> 

void swap (int *px, int *py)
{
    int temp=*px;
    *px=*py;
    *py=temp;
}


int main()
{
    using namespace std;
    int a=1, b=2;
    cout<<"before swap:"<<endl;
    cout<<"a="<<a<<",b="<<b<<endl;
    swap(&a, &b);
    cout<<"after swap:"<<endl;
    cout<<"a="<<a<<",b="<<b<<endl;
}