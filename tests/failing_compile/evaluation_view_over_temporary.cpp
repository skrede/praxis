#include "praxis/evaluation.h"

#include <array>

namespace {

double origin()
{
    return 0.0;
}

struct probe_ops
{
    double (*offset)() = &origin;
};

praxis::evaluation::case_result unanswered(const void *, const void *, praxis::evaluation::case_source &, const praxis::evaluation::tolerance_pair &)
{
    return praxis::evaluation::case_result{praxis::evaluation::agreement::not_exercised, praxis::evaluation::residual{praxis::evaluation::residual_kind::element_wise, 0.0, 0.0}};
}

constexpr std::array probe_table{
        praxis::evaluation::slot_evaluation{"probe.offset", praxis::evaluation::residual_kind::element_wise,
                                            praxis::evaluation::tolerance_of(praxis::evaluation::residual_kind::element_wise), &unanswered},
};

constexpr praxis::evaluation::capability_evaluations<probe_ops> described_probes{"probe", probe_table};

}

namespace praxis::evaluation::probe {

evaluation_view view_over_a_temporary()
{
    const probe_ops held{};

    return evaluation_view::of(probe_ops{}, held, described_probes);
}

}
