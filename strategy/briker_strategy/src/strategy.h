#ifndef STRATEGY_H
#define STRATEGY_H

#include<iostream>

namespace unit
{
    class Strategy
    {
        private:
                /*data*/
        public:
                Strategy(){}
                virtual ~Strategy(){}
        protected:
                bool enable_;
    };
    
}

#endif