#ifndef __VERIFY_ROBUST_CHANGE_H
#define __VERIFY_ROBUST_CHANGE_H

#include"semantic_mesh_localization/verify_cal_likelihood.h"

namespace semlocali{

    class VerRobust : public VerCalLike{

        public:
            VerRobust();

            virtual ~VerRobust();

            bool setup_robust( ros::NodeHandle& node, ros::NodeHandle& privateNode);

            void spin_robust();




    };
}

#endif
