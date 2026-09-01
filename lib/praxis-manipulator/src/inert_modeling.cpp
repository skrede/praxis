#include "praxis/manipulator/modeling.h"

namespace praxis::manipulator::inert {

expected<screw_chain, refusal> build_chain(const meios::model<> &)
{
    return unexpected(refusal::not_implemented);
}

}
