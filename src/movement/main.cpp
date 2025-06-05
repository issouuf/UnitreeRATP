#include <iostream>
#include <string>

#include  "libs/Movement/implementations/Unitree/dogs/Unitree Go 1/UnitreeGo1.hpp"

using namespace std;
using namespace std::chrono;

int main(int argc, char *argv[])
{
    UnitreeGo1 robot = UnitreeGo1();
    try
    {
        robot.Connect();
        robot.Start();
        robot.MoveXYZ(500, 0, 30);
    }
    catch (const mqtt::exception &exc)
    {
        robot.Stop();
        robot.Disconnect();
        cerr << exc.what() << endl;
        return 1;
    }

    return 0;
}
