#ifndef FUZZY_CONTROL_H
#define FUZZY_CONTROL_H

#include <fl/Headers.h>

#define PI 3.14159265

struct output {
    float direction;
    float speed;
};

class fuzzy_control
{
public:
    virtual ~fuzzy_control();
    virtual void init_controller() = 0;
    virtual void process();
    virtual void setValues();
    virtual output getOutput();

protected:
    fl::Engine* p_engine;
    fl::OutputVariable* p_driSpeed;
    fl::OutputVariable* p_driDir;
};

class obstacle_avoidance : public fuzzy_control
{
public:
    void init_controller();
    void setValues(float obstDist, float obstDir);

private:
    //Inputvariables
    fl::InputVariable*  p_obstDist;
    fl::InputVariable*  p_obstDir;
};

class take_marble : public fuzzy_control
{
public:
    void init_controller();
    void setValues(float obstacleDist, float obstacleDir, float marbleDist);

private:
    //Inputvariables
    fl::InputVariable*  p_obstacleDist;
    fl::InputVariable*  p_obstacleDir;
    fl::InputVariable*  p_marbleDist;
};

#endif // FUZZY_CONTROL_H
