/*
本程序功能：
    1、提供一个基于 ncurses 的无人机集群编队控制 TUI；
    2、支持起飞/降落/悬停/返航快捷指令；
    3、支持发送新版 Formation.msg 中的静态/动态阵型；
    4、支持通过网格编辑 STATIC_FORMATION_CUSTOM 的相对偏移量。
*/
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <ncurses.h>
#include <ros/ros.h>
#include <sstream>
#include <string>
#include <sunray_msgs/Formation.h>
#include <sunray_msgs/UAVSwarmCMD.h>
#include <vector>

namespace
{

constexpr int kGridSize = 21;
constexpr int kLeaderIndex = kGridSize / 2;
constexpr int kCellWidth = 2;

struct TuiConfig
{
    std::string swarm_cmd_topic{"/sunray/swarm/uav_swarm_cmd"};
    int agent_id{99};
    int swarm_num{3};
    double grid_resolution{1.0};
};

struct FormationDefaults
{
    sunray_msgs::Formation cmd{};
};

struct GridState
{
    int cursor_row{kLeaderIndex};
    int cursor_col{kLeaderIndex};
    std::vector<std::vector<bool>> selected{
        static_cast<size_t>(kGridSize), std::vector<bool>(static_cast<size_t>(kGridSize), false)};
};

struct AppState
{
    TuiConfig config{};
    FormationDefaults defaults{};
    GridState grid{};
    std::string status{"Ready"};
};

std::string formatDouble(const double value)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2) << value;
    return oss.str();
}

const char *formationTypeName(const uint8_t formation_type)
{
    switch (formation_type)
    {
    case sunray_msgs::Formation::STATIC_KEEP_FORMATION:
        return "STATIC_KEEP_FORMATION";
    case sunray_msgs::Formation::STATIC_FORMATION_LINE:
        return "STATIC_FORMATION_LINE";
    case sunray_msgs::Formation::STATIC_FORMATION_POLYGON:
        return "STATIC_FORMATION_POLYGON";
    case sunray_msgs::Formation::STATIC_FORMATION_RANDOM:
        return "STATIC_FORMATION_RANDOM";
    case sunray_msgs::Formation::STATIC_FORMATION_CUSTOM:
        return "STATIC_FORMATION_CUSTOM";
    case sunray_msgs::Formation::DYNAMIC_FORMATION_RING:
        return "DYNAMIC_FORMATION_RING";
    case sunray_msgs::Formation::DYNAMIC_FORMATION_POLYGON:
        return "DYNAMIC_FORMATION_POLYGON";
    case sunray_msgs::Formation::DYNAMIC_FORMATION_LEMNISCATE:
        return "DYNAMIC_FORMATION_LEMNISCATE";
    default:
        return "UNKNOWN";
    }
}

void setStatus(AppState &state, const std::string &status)
{
    state.status = status;
}

template <typename T>
bool parseValue(const std::string &text, T &value)
{
    std::istringstream iss(text);
    iss >> value;
    return iss && iss.eof();
}

std::string promptLine(const std::string &label)
{
    int rows = 0;
    int cols = 0;
    getmaxyx(stdscr, rows, cols);
    move(rows - 2, 0);
    clrtoeol();
    mvprintw(rows - 2, 0, "%s", label.c_str());
    echo();
    curs_set(1);
    timeout(-1);

    char buffer[256] = {0};
    getnstr(buffer, sizeof(buffer) - 1);

    noecho();
    curs_set(0);
    timeout(50);
    (void)cols;
    return std::string(buffer);
}

template <typename T>
T promptValue(const std::string &label, const T &default_value)
{
    const std::string input = promptLine(label + " [" + formatDouble(static_cast<double>(default_value)) + "]: ");
    if (input.empty())
    {
        return default_value;
    }

    T value{};
    if (!parseValue(input, value))
    {
        return default_value;
    }
    return value;
}

template <>
int promptValue<int>(const std::string &label, const int &default_value)
{
    const std::string input = promptLine(label + " [" + std::to_string(default_value) + "]: ");
    if (input.empty())
    {
        return default_value;
    }

    int value = default_value;
    if (!parseValue(input, value))
    {
        return default_value;
    }
    return value;
}

int countSelected(const GridState &grid)
{
    int count = 0;
    for (const std::vector<bool> &row : grid.selected)
    {
        count += static_cast<int>(std::count(row.begin(), row.end(), true));
    }
    return count;
}

void clearGrid(GridState &grid)
{
    for (std::vector<bool> &row : grid.selected)
    {
        std::fill(row.begin(), row.end(), false);
    }
}

void drawKeyLine(const int y, const int x, const char *key, const char *desc)
{
    attron(COLOR_PAIR(4));
    mvprintw(y, x, "%s", key);
    attroff(COLOR_PAIR(4));
    mvprintw(y, x + static_cast<int>(std::string(key).size()), " %s", desc);
}

void drawGrid(const AppState &state, const int start_y, const int start_x)
{
    for (int r = 0; r < kGridSize; ++r)
    {
        for (int c = 0; c < kGridSize; ++c)
        {
            const bool is_leader = (r == kLeaderIndex && c == kLeaderIndex);
            const bool is_selected = state.grid.selected[static_cast<size_t>(r)][static_cast<size_t>(c)];
            const bool is_cursor = (r == state.grid.cursor_row && c == state.grid.cursor_col);

            char ch = '.';
            int color = 1;
            if (is_leader)
            {
                ch = 'L';
                color = 3;
            }
            else if (is_selected)
            {
                ch = 'O';
                color = 2;
            }

            if (is_cursor)
            {
                attron(A_REVERSE);
            }
            attron(COLOR_PAIR(color));
            mvaddch(start_y + r, start_x + c * kCellWidth, ch);
            mvaddch(start_y + r, start_x + c * kCellWidth + 1, ' ');
            attroff(COLOR_PAIR(color));
            if (is_cursor)
            {
                attroff(A_REVERSE);
            }
        }
    }
}

void drawUi(const AppState &state)
{
    clear();
    const int grid_x = 2;
    const int grid_y = 2;
    const int info_x = grid_x + kGridSize * kCellWidth + 4;
    int line = 1;

    attron(COLOR_PAIR(3));
    mvprintw(0, 2, "Sunray UAV Formation TUI");
    attroff(COLOR_PAIR(3));
    mvprintw(0, 32, "topic=%s", state.config.swarm_cmd_topic.c_str());

    drawGrid(state, grid_y, grid_x);

    mvprintw(line++, info_x, "Basic:");
    mvprintw(line++, info_x, "agent_id=%d  swarm_num=%d", state.config.agent_id, state.config.swarm_num);
    mvprintw(line++, info_x, "grid_resolution=%.2f m/cell", state.config.grid_resolution);
    mvprintw(line++, info_x, "selected_custom_offsets=%d", countSelected(state.grid));
    line++;

    mvprintw(line++, info_x, "Leader:");
    mvprintw(line++, info_x, "pos=(%.2f, %.2f, %.2f)",
             state.defaults.cmd.leader_pos.x,
             state.defaults.cmd.leader_pos.y,
             state.defaults.cmd.leader_pos.z);
    mvprintw(line++, info_x, "yaw=%.2f rad", state.defaults.cmd.leader_yaw);
    line++;

    mvprintw(line++, info_x, "Formation defaults:");
    mvprintw(line++, info_x, "line: spacing=%.2f angle=%.2fdeg",
             state.defaults.cmd.static_line_spacing,
             state.defaults.cmd.static_line_angle);
    mvprintw(line++, info_x, "polygon: spacing=%.2f", state.defaults.cmd.static_polygon_spacing);
    mvprintw(line++, info_x, "ring: r=%.2f v=%.2f t=%.2f",
             state.defaults.cmd.dynamic_ring_radius,
             state.defaults.cmd.dynamic_ring_move_speed,
             state.defaults.cmd.dynamic_time);
    mvprintw(line++, info_x, "dyn_polygon: spacing=%.2f v=%.2f",
             state.defaults.cmd.dynamic_polygon_spacing,
             state.defaults.cmd.dynamic_polygon_move_speed);
    mvprintw(line++, info_x, "lemniscate: x=%.2f y=%.2f v=%.2f",
             state.defaults.cmd.dynamic_lemniscate_x_radius,
             state.defaults.cmd.dynamic_lemniscate_y_radius,
             state.defaults.cmd.dynamic_lemniscate_move_speed);
    line++;

    mvprintw(line++, info_x, "Keys:");
    drawKeyLine(line++, info_x, "arrows/wasd", "move cursor");
    drawKeyLine(line++, info_x, "space", "toggle custom offset");
    drawKeyLine(line++, info_x, "x", "clear custom offsets");
    drawKeyLine(line++, info_x, "S/L", "save/load custom grid");
    drawKeyLine(line++, info_x, "A/N/G", "edit agent_id/swarm_num/grid");
    drawKeyLine(line++, info_x, "E", "edit leader and formation params");
    drawKeyLine(line++, info_x, "t/g/h/b", "takeoff/land/hover/return");
    drawKeyLine(line++, info_x, "0", "STATIC_KEEP_FORMATION");
    drawKeyLine(line++, info_x, "1", "STATIC_FORMATION_LINE");
    drawKeyLine(line++, info_x, "2", "STATIC_FORMATION_POLYGON");
    drawKeyLine(line++, info_x, "3", "STATIC_FORMATION_RANDOM");
    drawKeyLine(line++, info_x, "9", "STATIC_FORMATION_CUSTOM from grid");
    drawKeyLine(line++, info_x, "r", "DYNAMIC_FORMATION_RING");
    drawKeyLine(line++, info_x, "p", "DYNAMIC_FORMATION_POLYGON");
    drawKeyLine(line++, info_x, "8", "DYNAMIC_FORMATION_LEMNISCATE");
    drawKeyLine(line++, info_x, "q", "quit");

    int rows = 0;
    int cols = 0;
    getmaxyx(stdscr, rows, cols);
    attron(COLOR_PAIR(4));
    mvprintw(rows - 4, 2, "Status:");
    attroff(COLOR_PAIR(4));
    mvprintw(rows - 4, 10, "%s", state.status.c_str());
    (void)cols;
    refresh();
}

std::vector<geometry_msgs::Point> collectCustomOffsets(const GridState &grid, const double resolution)
{
    std::vector<geometry_msgs::Point> offsets;
    offsets.reserve(static_cast<size_t>(countSelected(grid)));
    for (int r = 0; r < kGridSize; ++r)
    {
        for (int c = 0; c < kGridSize; ++c)
        {
            if (r == kLeaderIndex && c == kLeaderIndex)
            {
                continue;
            }
            if (!grid.selected[static_cast<size_t>(r)][static_cast<size_t>(c)])
            {
                continue;
            }

            geometry_msgs::Point offset;
            offset.x = static_cast<double>(kLeaderIndex - r) * resolution;
            offset.y = static_cast<double>(kLeaderIndex - c) * resolution;
            offset.z = 0.0;
            offsets.push_back(offset);
        }
    }
    return offsets;
}

bool saveGrid(const std::string &path, const GridState &grid)
{
    std::ofstream out(path);
    if (!out)
    {
        return false;
    }

    out << "formation_tui_grid " << kGridSize << "\n";
    for (int r = 0; r < kGridSize; ++r)
    {
        for (int c = 0; c < kGridSize; ++c)
        {
            if (grid.selected[static_cast<size_t>(r)][static_cast<size_t>(c)])
            {
                out << r << " " << c << "\n";
            }
        }
    }
    return true;
}

bool loadGrid(const std::string &path, GridState &grid)
{
    std::ifstream in(path);
    if (!in)
    {
        return false;
    }

    clearGrid(grid);
    std::string header;
    int size = 0;
    in >> header >> size;
    if (header != "formation_tui_grid" || size != kGridSize)
    {
        return false;
    }

    int r = 0;
    int c = 0;
    while (in >> r >> c)
    {
        if (r >= 0 && r < kGridSize && c >= 0 && c < kGridSize && !(r == kLeaderIndex && c == kLeaderIndex))
        {
            grid.selected[static_cast<size_t>(r)][static_cast<size_t>(c)] = true;
        }
    }
    return true;
}

sunray_msgs::Formation makeFormationFromDefaults(const AppState &state, const uint8_t formation_type)
{
    sunray_msgs::Formation cmd = state.defaults.cmd;
    cmd.formation_type = formation_type;
    return cmd;
}

void publishSwarmCmd(ros::Publisher &pub, const AppState &state, const uint8_t swarm_cmd, sunray_msgs::Formation formation)
{
    sunray_msgs::UAVSwarmCMD msg;
    msg.header.stamp = ros::Time::now();
    msg.cmd_source = sunray_msgs::UAVSwarmCMD::TERMINAL;
    msg.agent_id = static_cast<uint8_t>(std::max(0, state.config.agent_id));
    msg.swarm_cmd = swarm_cmd;
    formation.header.stamp = msg.header.stamp;
    msg.formation_cmd = formation;
    pub.publish(msg);
    ros::spinOnce();
}

void publishSimpleCmd(ros::Publisher &pub, AppState &state, const uint8_t swarm_cmd, const std::string &name)
{
    publishSwarmCmd(pub, state, swarm_cmd, state.defaults.cmd);
    setStatus(state, "Sent " + name);
}

void publishFormation(ros::Publisher &pub, AppState &state, const uint8_t formation_type)
{
    sunray_msgs::Formation cmd = makeFormationFromDefaults(state, formation_type);

    if (formation_type == sunray_msgs::Formation::STATIC_FORMATION_CUSTOM)
    {
        const int selected_count = countSelected(state.grid);
        if (selected_count != state.config.swarm_num)
        {
            setStatus(state, "CUSTOM requires selected offsets == swarm_num");
            return;
        }

        cmd.custom_offsets_pos = collectCustomOffsets(state.grid, state.config.grid_resolution);
        cmd.custom_offsets_yaw.assign(cmd.custom_offsets_pos.size(), 0.0F);
    }

    publishSwarmCmd(pub, state, sunray_msgs::UAVSwarmCMD::SWARM_FORMATION, cmd);
    setStatus(state, std::string("Sent ") + formationTypeName(formation_type));
}

void editConfig(AppState &state)
{
    state.config.agent_id = promptValue<int>("agent_id, 99 means broadcast", state.config.agent_id);
    state.config.swarm_num = std::max(1, promptValue<int>("swarm_num / custom offset count", state.config.swarm_num));
}

void editGridResolution(AppState &state)
{
    state.config.grid_resolution = std::max(0.01, promptValue<double>("grid_resolution(m/cell)", state.config.grid_resolution));
}

void editFormationDefaults(AppState &state)
{
    sunray_msgs::Formation &cmd = state.defaults.cmd;
    cmd.leader_pos.x = promptValue<double>("leader_x(m)", cmd.leader_pos.x);
    cmd.leader_pos.y = promptValue<double>("leader_y(m)", cmd.leader_pos.y);
    cmd.leader_pos.z = promptValue<double>("leader_z(m)", cmd.leader_pos.z);
    cmd.leader_yaw = promptValue<float>("leader_yaw(rad)", cmd.leader_yaw);
    cmd.dynamic_time = promptValue<float>("dynamic_time(s)", cmd.dynamic_time);

    cmd.static_line_spacing = promptValue<float>("static_line_spacing(m)", cmd.static_line_spacing);
    cmd.static_line_angle = promptValue<float>("static_line_angle(deg)", cmd.static_line_angle);
    cmd.static_polygon_spacing = promptValue<float>("static_polygon_spacing(m)", cmd.static_polygon_spacing);

    cmd.dynamic_ring_radius = promptValue<float>("dynamic_ring_radius(m)", cmd.dynamic_ring_radius);
    cmd.dynamic_ring_move_speed = promptValue<float>("dynamic_ring_move_speed(m/s)", cmd.dynamic_ring_move_speed);
    cmd.dynamic_polygon_spacing = promptValue<float>("dynamic_polygon_spacing(m)", cmd.dynamic_polygon_spacing);
    cmd.dynamic_polygon_move_speed = promptValue<float>("dynamic_polygon_move_speed(m/s)", cmd.dynamic_polygon_move_speed);
    cmd.dynamic_lemniscate_x_radius =
        promptValue<float>("dynamic_lemniscate_x_radius(m)", cmd.dynamic_lemniscate_x_radius);
    cmd.dynamic_lemniscate_y_radius =
        promptValue<float>("dynamic_lemniscate_y_radius(m)", cmd.dynamic_lemniscate_y_radius);
    cmd.dynamic_lemniscate_move_speed =
        promptValue<float>("dynamic_lemniscate_move_speed(m/s)", cmd.dynamic_lemniscate_move_speed);
}

void loadParams(ros::NodeHandle &nh, AppState &state)
{
    nh.param<std::string>("swarm_cmd_topic", state.config.swarm_cmd_topic, "/sunray/swarm/uav_swarm_cmd");
    nh.param("default_agent_id", state.config.agent_id, 99);
    nh.param("swarm_num", state.config.swarm_num, 3);
    nh.param("grid_resolution", state.config.grid_resolution, 1.0);

    sunray_msgs::Formation &cmd = state.defaults.cmd;
    cmd.formation_type = sunray_msgs::Formation::STATIC_FORMATION_LINE;
    nh.param("default_leader_x", cmd.leader_pos.x, 0.0 /*米*/);
    nh.param("default_leader_y", cmd.leader_pos.y, 0.0 /*米*/);
    nh.param("default_leader_z", cmd.leader_pos.z, 1.5 /*米*/);
    nh.param("default_leader_yaw", cmd.leader_yaw, 0.0F /*rad*/);
    nh.param("default_dynamic_time", cmd.dynamic_time, 10.0F /*秒*/);
    nh.param("default_static_line_spacing", cmd.static_line_spacing, 1.5F /*米*/);
    nh.param("default_static_line_angle", cmd.static_line_angle, 0.0F /*deg*/);
    nh.param("default_static_polygon_spacing", cmd.static_polygon_spacing, 2.0F /*米*/);
    nh.param("default_dynamic_ring_radius", cmd.dynamic_ring_radius, 2.0F /*米*/);
    nh.param("default_dynamic_ring_move_speed", cmd.dynamic_ring_move_speed, 0.5F /*米/秒*/);
    nh.param("default_dynamic_polygon_spacing", cmd.dynamic_polygon_spacing, 2.0F /*米*/);
    nh.param("default_dynamic_polygon_move_speed", cmd.dynamic_polygon_move_speed, 0.5F /*米/秒*/);
    nh.param("default_dynamic_lemniscate_x_radius", cmd.dynamic_lemniscate_x_radius, 3.0F /*米*/);
    nh.param("default_dynamic_lemniscate_y_radius", cmd.dynamic_lemniscate_y_radius, 2.0F /*米*/);
    nh.param("default_dynamic_lemniscate_move_speed", cmd.dynamic_lemniscate_move_speed, 0.5F /*米/秒*/);
}

void initCurses()
{
    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    timeout(50);
    curs_set(0);

    if (has_colors())
    {
        start_color();
        init_pair(1, COLOR_WHITE, COLOR_BLACK);
        init_pair(2, COLOR_GREEN, COLOR_BLACK);
        init_pair(3, COLOR_CYAN, COLOR_BLACK);
        init_pair(4, COLOR_YELLOW, COLOR_BLACK);
    }
}

} // namespace

int main(int argc, char **argv)
{
    ros::init(argc, argv, "formation_tui");
    ros::NodeHandle nh("~");

    AppState state;
    loadParams(nh, state);
    ros::Publisher swarm_pub = nh.advertise<sunray_msgs::UAVSwarmCMD>(state.config.swarm_cmd_topic, 10);

    initCurses();

    while (ros::ok())
    {
        ros::spinOnce();
        drawUi(state);

        const int ch = getch();
        if (ch == ERR)
        {
            continue;
        }

        if (ch == KEY_UP || ch == 'w')
        {
            state.grid.cursor_row = std::max(0, state.grid.cursor_row - 1);
        }
        else if (ch == KEY_DOWN || ch == 's')
        {
            state.grid.cursor_row = std::min(kGridSize - 1, state.grid.cursor_row + 1);
        }
        else if (ch == KEY_LEFT || ch == 'a')
        {
            state.grid.cursor_col = std::max(0, state.grid.cursor_col - 1);
        }
        else if (ch == KEY_RIGHT || ch == 'd')
        {
            state.grid.cursor_col = std::min(kGridSize - 1, state.grid.cursor_col + 1);
        }
        else if (ch == ' ')
        {
            const int r = state.grid.cursor_row;
            const int c = state.grid.cursor_col;
            if (!(r == kLeaderIndex && c == kLeaderIndex))
            {
                state.grid.selected[static_cast<size_t>(r)][static_cast<size_t>(c)] =
                    !state.grid.selected[static_cast<size_t>(r)][static_cast<size_t>(c)];
            }
        }
        else if (ch == 'x')
        {
            clearGrid(state.grid);
            setStatus(state, "Custom grid cleared");
        }
        else if (ch == 'S')
        {
            std::string path = promptLine("Save grid file [custom_formation_grid.txt]: ");
            if (path.empty())
            {
                path = "custom_formation_grid.txt";
            }
            setStatus(state, saveGrid(path, state.grid) ? ("Saved " + path) : "Save failed");
        }
        else if (ch == 'L' || ch == 'l')
        {
            std::string path = promptLine("Load grid file [custom_formation_grid.txt]: ");
            if (path.empty())
            {
                path = "custom_formation_grid.txt";
            }
            setStatus(state, loadGrid(path, state.grid) ? ("Loaded " + path) : "Load failed");
        }
        else if (ch == 'A' || ch == 'N')
        {
            editConfig(state);
            setStatus(state, "Basic config updated");
        }
        else if (ch == 'G')
        {
            editGridResolution(state);
            setStatus(state, "Grid resolution updated");
        }
        else if (ch == 'E')
        {
            editFormationDefaults(state);
            setStatus(state, "Formation defaults updated");
        }
        else if (ch == 't')
        {
            publishSimpleCmd(swarm_pub, state, sunray_msgs::UAVSwarmCMD::SWARM_TAKEOFF, "SWARM_TAKEOFF");
        }
        else if (ch == 'g')
        {
            publishSimpleCmd(swarm_pub, state, sunray_msgs::UAVSwarmCMD::SWARM_LAND, "SWARM_LAND");
        }
        else if (ch == 'h')
        {
            publishSimpleCmd(swarm_pub, state, sunray_msgs::UAVSwarmCMD::SWARM_HOVER, "SWARM_HOVER");
        }
        else if (ch == 'b')
        {
            publishSimpleCmd(swarm_pub, state, sunray_msgs::UAVSwarmCMD::SWARM_RETURN, "SWARM_RETURN");
        }
        else if (ch == '0')
        {
            publishFormation(swarm_pub, state, sunray_msgs::Formation::STATIC_KEEP_FORMATION);
        }
        else if (ch == '1')
        {
            publishFormation(swarm_pub, state, sunray_msgs::Formation::STATIC_FORMATION_LINE);
        }
        else if (ch == '2')
        {
            publishFormation(swarm_pub, state, sunray_msgs::Formation::STATIC_FORMATION_POLYGON);
        }
        else if (ch == '3')
        {
            publishFormation(swarm_pub, state, sunray_msgs::Formation::STATIC_FORMATION_RANDOM);
        }
        else if (ch == '9')
        {
            publishFormation(swarm_pub, state, sunray_msgs::Formation::STATIC_FORMATION_CUSTOM);
        }
        else if (ch == 'r')
        {
            publishFormation(swarm_pub, state, sunray_msgs::Formation::DYNAMIC_FORMATION_RING);
        }
        else if (ch == 'p')
        {
            publishFormation(swarm_pub, state, sunray_msgs::Formation::DYNAMIC_FORMATION_POLYGON);
        }
        else if (ch == '8')
        {
            publishFormation(swarm_pub, state, sunray_msgs::Formation::DYNAMIC_FORMATION_LEMNISCATE);
        }
        else if (ch == 'q')
        {
            break;
        }
    }

    endwin();
    return 0;
}
