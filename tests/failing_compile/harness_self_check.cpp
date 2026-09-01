namespace praxis::probe {

struct rejected
{
};

int accept(const rejected &value);
int accept(rejected &&) = delete;

int harness_self_check()
{
    return accept(rejected{});
}

}
