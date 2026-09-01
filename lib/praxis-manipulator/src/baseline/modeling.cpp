#include "praxis/manipulator/baseline/modeling.h"
#include "praxis/manipulator/screw_chain_builder.h"

namespace praxis::manipulator {

expected<screw_chain, refusal> build_chain(const meios::model<> &model)
{
    return build_screw_chain(model);
}

}
