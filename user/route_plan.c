#include "route_plan.h"

// 路线规划模块只维护“当前走到哪一项”，
// 不承担默认值兼容、动作决策或参数派生逻辑。

static const route_plan_entry_t *g_route_plan = NULL;
static volatile uint8_t g_route_count = 0u;
static volatile uint8_t g_route_index = 0u;

void route_plan_set(const route_plan_entry_t *plan, uint8_t count)
{
    g_route_plan = plan;
    g_route_count = count;
    g_route_index = 0u;
}

void route_plan_reset(void)
{
    g_route_index = 0u;
}

bool route_plan_has_next(void)
{
    if ((g_route_plan == NULL) || (g_route_count == 0u))
    {
        return false;
    }

    return (g_route_index < g_route_count);
}

bool route_plan_peek(route_plan_entry_t *out_entry)
{
    if ((out_entry == NULL) || !route_plan_has_next())
    {
        return false;
    }

    *out_entry = g_route_plan[g_route_index];
    return true;
}

bool route_plan_peek_at(uint8_t index, route_plan_entry_t *out_entry)
{
    if ((out_entry == NULL) || (g_route_plan == NULL) || (index >= g_route_count))
    {
        return false;
    }

    *out_entry = g_route_plan[index];
    return true;
}

bool route_plan_commit(void)
{
    if (!route_plan_has_next())
    {
        return false;
    }

    if (g_route_index < 255u)
    {
        g_route_index++;
    }
    return true;
}

uint8_t route_plan_get_index(void)
{
    return g_route_index;
}

bool route_plan_is_element(route_turn_t turn)
{
    return (turn >= ROUTE_ELEM_BUTTON) && (turn <= ROUTE_ELEM_DIODE_FORK_R_TURN);
}

uint8_t route_plan_get_element_index(route_turn_t turn)
{
    if (!route_plan_is_element(turn))
    {
        return 0u;
    }

    return (uint8_t)((uint8_t)turn - (uint8_t)ROUTE_ELEM_BUTTON + 1u);
}
