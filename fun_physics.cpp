#include <cmath>
#include "fun_physics.hpp"

// helper functions
static ImVec2 operator+(ImVec2 const &a, ImVec2 const &b) noexcept { return {a.x + b.x, a.y + b.y}; }
static ImVec2 operator-(ImVec2 const &a, ImVec2 const &b) noexcept { return {a.x - b.x, a.y - b.y}; }

static ImVec2 operator*(ImVec2 const &a, float b) noexcept { return {a.x * b, a.y * b}; }
static ImVec2 operator*(float b, ImVec2 const &a) noexcept { return {b * a.x, b * a.y}; }

static ImVec2 operator/(ImVec2 const &a, float b) noexcept { return {a.x / b, a.y / b}; }

static float dot(ImVec2 const &a, ImVec2 const &b) noexcept
{
    return a.x * b.x + a.y * b.y;
}

static ImVec2 normal(ImVec2 const &vec) noexcept
{
    float const len = sqrtf(vec.x * vec.x + vec.y * vec.y);
    return vec / len;
}

ImVec2 fun::phys::rigid_body::center() const noexcept
{
    return (aabb.min + aabb.max) * .5f;
}

fun::phys::world::world(float gravity_) noexcept
    : gravity{gravity_} {}

fun::phys::object fun::phys::world::add_object(fun::phys::rigid_body const &body) noexcept
{
    // if (used.empty())
    // {
    id.reserve(next + 1);
    object.reserve(next + 1);
    vel.reserve(next + 1);
    force.reserve(next + 1);

    id.insert(id.begin() + next, next);
    object.insert(object.begin() + next, body);

    vel.emplace(vel.begin() + next);
    force.emplace(force.begin() + next);

    return next++;
    // }
    // else
    // {
    //     fun::phys::object ret = used.back();
    //     used.pop_back();

    //     id.reserve(ret + 1);
    //     object.reserve(ret + 1);
    //     vel.reserve(ret + 1);
    //     force.reserve(ret + 1);

    //     id.insert(id.begin() + ret, ret);
    //     object.insert(object.begin() + ret, body);

    //     vel.emplace(vel.begin() + ret, body.vel);
    //     force.emplace(force.begin() + ret);

    //     return ret;
    // }
}

void fun::phys::world::remove_object(fun::phys::object const &obj) noexcept
{
    used.push_back(*id.erase(std::ranges::find(id, obj)));
}

bool fun::phys::world::colliding(fun::phys::object const &id1, fun::phys::object const &id2) noexcept
{
    collision coll{
        .objects = {id1, id2},
    };
    ImVec2 const n = object[id2].center() - object[id1].center();
    float const a_halfw = object[id1].aabb.w() * .5f;
    float const b_halfw = object[id2].aabb.w() * .5f;

    if (float const x_overlap = a_halfw + b_halfw - std::fabsf(n.x);
        x_overlap > 0.f)
    {
        float const a_halfh = object[id1].aabb.h() * .5f;
        float const b_halfh = object[id2].aabb.h() * .5f;

        float y_overlap = a_halfh + b_halfh - std::fabsf(n.y);

        if (y_overlap > 0.f)
        {
            if (x_overlap > y_overlap)
            {
                if (n.x < 0.f)
                    coll.normal = ImVec2{-1.f, 0.f};
                else
                    coll.normal = ImVec2{};
                coll.pen = x_overlap;
                collisions.push(coll);
                return true;
            }
            else
            {
                if (n.y < 0.f)
                    coll.normal = ImVec2{0.f, -1.f};
                else
                    coll.normal = ImVec2{0.f, 1.f};
                coll.pen = y_overlap;
                collisions.push(coll);
                return true;
            }
        }
    }

    return false;
}

void fun::phys::world::integrate(fun::phys::object const &id, float delta) noexcept
{
    if (object[id].state == fun::phys::rigid_body::state_t::dynamic)
    {
        ImVec2 accel = (global_force + force[id]) * object[id].inv_mass;
        accel.y += gravity * object[id].inv_mass;

        vel[id] = vel[id] + accel * delta;

        object[id].aabb.min = object[id].aabb.min + vel[id] * delta;
        object[id].aabb.max = object[id].aabb.max + vel[id] * delta;
    }
}

void fun::phys::world::clear() noexcept
{
    used.assign(id.begin(), id.end());
    id.clear();
    next = 0;
}

void fun::phys::world::resolve(fun::phys::collision const &coll) noexcept
{
    fun::phys::object const &id1 = coll.objects[0];
    fun::phys::object const &id2 = coll.objects[1];

    float const sep_speed = dot(vel[id2] - vel[id1], coll.normal);

    if (sep_speed > 0.f)
        return;

    float const e = std::min(object[id1].restitution, object[id2].restitution);

    float j = -(1.f + e) * sep_speed;
    j /= object[id1].inv_mass + object[id2].inv_mass;

    // clang-format off
    float const total_mass = 1.f/object[id1].inv_mass + 1.f / object[id2].inv_mass;
    // clang-format on

    ImVec2 const impulse = coll.normal * j;
    vel[id1] = vel[id1] - impulse * (total_mass / object[id1].inv_mass);
    vel[id1] = std::min(object[id1].max_vel, vel[id1],
                        [](auto const &left, auto const &right)
                        {
                            if (left.x < right.x)
                                return true;
                            return (left.y < right.y);
                        });

    vel[id2] = vel[id2] + impulse * (total_mass / object[id2].inv_mass);
    vel[id2] = std::min(object[id2].max_vel, vel[id2],
                        [](auto const &left, auto const &right)
                        {
                            if (left.x < right.x)
                                return true;
                            return (left.y < right.y);
                        });

    ImVec2 const correction = std::max(coll.pen - phys::slop, 0.f) /
                              (object[id1].inv_mass + object[id2].inv_mass) *
                              .2f * coll.normal;

    object[id1].aabb.min = object[id1].aabb.min - object[id1].inv_mass * correction;
    object[id1].aabb.max = object[id1].aabb.max - object[id1].inv_mass * correction;

    object[id2].aabb.min = object[id2].aabb.min + object[id2].inv_mass * correction;
    object[id2].aabb.max = object[id2].aabb.max + object[id2].inv_mass * correction;
}