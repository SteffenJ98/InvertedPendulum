

class controller{
    public:
        controller(int max, int min,float KP, float KI,float KD){
            _KP=KP;
            _KI=KI;
            _KD=KD;
            _max=max;
            _min=min;
        }

        void do_step(float desired_value, float present_value,float dt,float& Plog,float& Ilog,float& Dlog,float& dtLog,float& PIDOutput);

        


    private:
        float _KI;
        float _KP;
        float _KD;
        float _integral=0;
        float _pre_error=0.0;
        float _pre_derivative=0;
        int _max;
        float _dt_old=0.0;
        float _dt=0.015;
        int _min;
        
};