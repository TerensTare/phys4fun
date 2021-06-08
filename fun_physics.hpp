#pragma once

#include <optional>
#include <stack>
#include <vector>

#include <imgui.h>

namespace fun::inline phys
{
    inline constexpr float slop{0.01f}; /// < A constant used for positional correction.

    struct rigid_body final
    {
        ImVec2 center() const noexcept;

        struct aabb_t
        {
            ImVec2 min;
            ImVec2 max;

            inline float w() const noexcept { return max.x - min.x; }
            inline float h() const noexcept { return max.y - min.y; }
        } aabb;

        float inv_mass = 0.f;
        float restitution = .2f;
        ImVec2 vel = {};

        ImVec2 max_vel = {};
        ImVec2 max_accel = {};

        ImVec4 color;

        enum class state_t
        {
            fixed,
            dynamic
        } state = state_t::dynamic;
    };

    using object = std::uint32_t;

    struct collision final
    {
        alignas(ImVec2) float pen;
        ImVec2 normal;
        object objects[2];
    };

    struct world final
    {
        explicit world(float gravity_) noexcept;

        object add_object(rigid_body const &body) noexcept;
        void remove_object(object const &obj) noexcept;

        bool colliding(object const &id1, object const &id2) noexcept;

        void integrate(object const &id, float delta) noexcept;
        void resolve(collision const &coll) noexcept;

        void clear() noexcept;

        std::vector<object> id;
        std::vector<object> used;

        std::vector<rigid_body> object;

        std::vector<ImVec2> vel;
        std::vector<ImVec2> force;

        std::stack<collision> collisions;

        fun::phys::object next{};
        float gravity;
        ImVec2 global_force;
    };
}