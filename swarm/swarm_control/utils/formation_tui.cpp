/*
本程序功能：
    1、实现基于 ncurses 的编队交互界面，在 20x20 网格上编辑 follower 相对 Leader 的自定义位置
    2、支持保存/加载自定义阵型文件，并通过 UAVSwarmCMD.custom_offsets 发布自定义阵型
    3、集成 ring/line/column/v_shape/wedge、起飞、降落、悬停、返航和 leader 移动等快捷控制
*/
#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <ncurses.h>
#include <ros/ros.h>
#include <sstream>
#include <string>
#include <sunray_msgs/UAVSwarmCMD.h>
#include <vector>

namespace
{

constexpr int kRows = 20;
constexpr int kCols = 20;
constexpr int kLeaderRow = kRows / 2;
constexpr int kLeaderCol = kCols / 2;
constexpr int kCellWidth = 2;

struct GridState
{
    int cursor_row{kLeaderRow};
    int cursor_col{kLeaderCol};
    std::vector<std::vector<bool>> selected{static_cast<size_t>(kRows), std::vector<bool>(kCols, false)};
};

struct AppState
{
    GridState grid{};
    std::string status{};
    ros::Time status_time{};
    int agent_num{1};
};

int expectedSeatCount(const AppState &state)
{
    return state.agent_num;
}

void setStatus(AppState &state, const std::string &msg)
{
    state.status = msg;
    state.status_time = ros::Time::now();
}

std::string promptLine(const std::string &label)
{
    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    move(rows - 2, 0);
    clrtoeol();
    mvprintw(rows - 2, 0, "%s", label.c_str());
    timeout(-1);
    echo();
    curs_set(1);
    char buf[256];
    getnstr(buf, sizeof(buf) - 1);
    noecho();
    curs_set(0);
    timeout(50);
    return std::string(buf);
}

void drawGrid(const AppState &state, int start_y, int start_x)
{
    for (int r = 0; r < kRows; ++r)
    {
        for (int c = 0; c < kCols; ++c)
        {
            int y = start_y + r;
            int x = start_x + c * kCellWidth;
            bool is_leader = (r == kLeaderRow && c == kLeaderCol);
            bool is_cursor = (r == state.grid.cursor_row && c == state.grid.cursor_col);
            bool is_selected = state.grid.selected[r][c];
            chtype ch = '.';
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
            mvaddch(y, x, ch);
            mvaddch(y, x + 1, ' ');
            attroff(COLOR_PAIR(color));
            if (is_cursor)
            {
                attroff(A_REVERSE);
            }
        }
    }
}

void drawKeyLine(int y, int x, const char *key, const char *desc)
{
    if (has_colors())
    {
        attron(COLOR_PAIR(4));
        mvprintw(y, x, "%s", key);
        attroff(COLOR_PAIR(4));
        mvprintw(y, x + static_cast<int>(std::strlen(key)), " %s", desc);
    }
    else
    {
        mvprintw(y, x, "%s %s", key, desc);
    }
}

int countSelected(const GridState &grid);

void drawKeyPairLine(int y, int x, const char *key1, const char *desc1, const char *key2, const char *desc2)
{
    int offset = 0;
    if (has_colors())
    {
        attron(COLOR_PAIR(4));
        mvprintw(y, x, "%s", key1);
        attroff(COLOR_PAIR(4));
        offset += static_cast<int>(std::strlen(key1));
        mvprintw(y, x + offset, " %s   ", desc1);
        offset += static_cast<int>(std::strlen(desc1)) + 4;
        attron(COLOR_PAIR(4));
        mvprintw(y, x + offset, "%s", key2);
        attroff(COLOR_PAIR(4));
        offset += static_cast<int>(std::strlen(key2));
        mvprintw(y, x + offset, " %s", desc2);
    }
    else
    {
        mvprintw(y, x, "%s %s   %s %s", key1, desc1, key2, desc2);
    }
}

void drawUi(const AppState &state)
{
    clear();
    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    int grid_width = kCols * kCellWidth;
    int grid_start_y = 1;
    int grid_start_x = 2;

    mvprintw(0, 2, "Swarm Formation TUI (20x20, leader centered)");
    drawGrid(state, grid_start_y, grid_start_x);

    int info_x = grid_start_x + grid_width + 3;
    int line = 1;

    mvprintw(line++, info_x, "Controls:");
    drawKeyLine(line++, info_x, "Arrows/WASD:", "move cursor");
    drawKeyLine(line++, info_x, "Space:", "toggle seat");
    drawKeyLine(line++, info_x, "c:", "send custom formation");
    drawKeyLine(line++, info_x, "x:", "clear selection");
    drawKeyPairLine(line++, info_x, "S:", "save", "L:", "load");
    drawKeyLine(line++, info_x, "m:", "move leader goal");
    drawKeyPairLine(line++, info_x, "t:", "takeoff", "g:", "land");
    drawKeyLine(line++, info_x, "h:", "hover");
    drawKeyLine(line++, info_x, "b:", "return_home");
    drawKeyLine(line++, info_x, "1:", "ring");
    drawKeyLine(line++, info_x, "4:", "line");
    drawKeyLine(line++, info_x, "5:", "column");
    drawKeyPairLine(line++, info_x, "6:", "v_shape", "7:", "wedge");
    drawKeyPairLine(line++, info_x, "2:", "expand", "3:", "contract");
    drawKeyLine(line++, info_x, "q:", "quit");

    int selected_count = countSelected(state.grid);
    int follower_count = expectedSeatCount(state);
    line++;
    mvprintw(line++, info_x, "Selected seats: %d", selected_count);
    mvprintw(line++, info_x, "Followers: %d", follower_count);
    if (selected_count != follower_count)
    {
        mvprintw(line++, info_x, "Note: count mismatch");
    }

    int status_y = rows - 3;
    mvprintw(status_y, 2, "Status: %s", state.status.c_str());

    refresh();
}

std::vector<geometry_msgs::Point> collectOffsets(const GridState &grid)
{
    std::vector<geometry_msgs::Point> offsets;
    offsets.reserve(kRows * kCols);
    for (int r = 0; r < kRows; ++r)
    {
        for (int c = 0; c < kCols; ++c)
        {
            if (r == kLeaderRow && c == kLeaderCol)
            {
                continue;
            }
            if (!grid.selected[r][c])
            {
                continue;
            }
            int dx = kLeaderRow - r; // forward is +x (up)
            int dy = kLeaderCol - c; // left is +y (left)
            geometry_msgs::Point pt;
            pt.x = static_cast<double>(dx);
            pt.y = static_cast<double>(dy);
            pt.z = 0.0;
            offsets.push_back(pt);
        }
    }
    return offsets;
}

int countSelected(const GridState &grid)
{
    int count = 0;
    for (int r = 0; r < kRows; ++r)
    {
        for (int c = 0; c < kCols; ++c)
        {
            if (grid.selected[r][c])
            {
                ++count;
            }
        }
    }
    return count;
}

void clearSelection(GridState &grid)
{
    for (int r = 0; r < kRows; ++r)
    {
        for (int c = 0; c < kCols; ++c)
        {
            grid.selected[r][c] = false;
        }
    }
}

bool saveOffsets(const std::string &path, const GridState &grid)
{
    std::ofstream out(path);
    if (!out)
    {
        return false;
    }
    out << "grid " << kRows << " " << kCols << "\n";
    for (int r = 0; r < kRows; ++r)
    {
        for (int c = 0; c < kCols; ++c)
        {
            if (r == kLeaderRow && c == kLeaderCol)
            {
                continue;
            }
            if (!grid.selected[r][c])
            {
                continue;
            }
            int dx = kLeaderRow - r;
            int dy = kLeaderCol - c;
            out << "offset " << dx << " " << dy << "\n";
        }
    }
    return true;
}

bool loadOffsets(const std::string &path, GridState &grid)
{
    std::ifstream in(path);
    if (!in)
    {
        return false;
    }
    clearSelection(grid);
    std::string token;
    int rows = kRows;
    int cols = kCols;
    while (in >> token)
    {
        if (token == "grid")
        {
            in >> rows >> cols;
        }
        else if (token == "offset")
        {
            int dx = 0;
            int dy = 0;
            in >> dx >> dy;
            int r = kLeaderRow - dx;
            int c = kLeaderCol - dy;
            if (r >= 0 && r < kRows && c >= 0 && c < kCols)
            {
                grid.selected[r][c] = true;
            }
        }
    }
    (void)rows;
    (void)cols;
    return true;
}

void publishUAVSwarmCMD(ros::Publisher &pub, uint8_t swarm_cmd,
                        uint8_t formation = sunray_msgs::UAVSwarmCMD::KEEP_FORMATION, float formation_param = 0.0f,
                        double x = 0.0, double y = 0.0, double z = 0.0, double yaw = 0.0,
                        const std::vector<geometry_msgs::Point> &custom_offsets = {})
{
    sunray_msgs::UAVSwarmCMD msg;
    msg.header.stamp = ros::Time::now();
    msg.cmd_source = sunray_msgs::UAVSwarmCMD::EXTERNAL_MODULE;
    msg.swarm_cmd = swarm_cmd;
    msg.formation = formation;
    msg.formation_param = formation_param;
    msg.leader_pos.x = x;
    msg.leader_pos.y = y;
    msg.leader_pos.z = z;
    msg.leader_yaw = yaw;
    msg.custom_offsets = custom_offsets;
    pub.publish(msg);
}

} // namespace

int main(int argc, char **argv)
{
    ros::init(argc, argv, "formation_tui");
    ros::NodeHandle nh("~");

    ros::Publisher swarm_pub = nh.advertise<sunray_msgs::UAVSwarmCMD>("/sunray/swarm/uav_swarm_cmd", 10);

    AppState state;
    if (!nh.getParam("agent_num", state.agent_num))
    {
        if (!ros::param::get("agent_num", state.agent_num))
        {
            ros::param::param<int>("/agent_num", state.agent_num, 1);
        }
    }
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

    setStatus(state, "Ready");

    while (ros::ok())
    {
        ros::spinOnce();
        drawUi(state);

        int ch = getch();
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
            state.grid.cursor_row = std::min(kRows - 1, state.grid.cursor_row + 1);
        }
        else if (ch == KEY_LEFT || ch == 'a')
        {
            state.grid.cursor_col = std::max(0, state.grid.cursor_col - 1);
        }
        else if (ch == KEY_RIGHT || ch == 'd')
        {
            state.grid.cursor_col = std::min(kCols - 1, state.grid.cursor_col + 1);
        }
        else if (ch == ' ')
        {
            int r = state.grid.cursor_row;
            int c = state.grid.cursor_col;
            if (!(r == kLeaderRow && c == kLeaderCol))
            {
                if (!state.grid.selected[r][c])
                {
                    int follower_count = expectedSeatCount(state);
                    int selected_count = countSelected(state.grid);
                    if (selected_count >= follower_count)
                    {
                        setStatus(state, "Selected seats exceed follower count");
                        continue;
                    }
                }
                state.grid.selected[r][c] = !state.grid.selected[r][c];
            }
        }
        else if (ch == 'x')
        {
            clearSelection(state.grid);
            setStatus(state, "Selection cleared");
        }
        else if (ch == 'S')
        {
            std::string path = promptLine("Save file (default custom_formation.txt): ");
            if (path.empty())
            {
                path = "custom_formation.txt";
            }
            if (saveOffsets(path, state.grid))
            {
                setStatus(state, "Saved to " + path);
            }
            else
            {
                setStatus(state, "Save failed");
            }
        }
        else if (ch == 'L' || ch == 'l')
        {
            std::string path = promptLine("Load file (default custom_formation.txt): ");
            if (path.empty())
            {
                path = "custom_formation.txt";
            }
            if (loadOffsets(path, state.grid))
            {
                setStatus(state, "Loaded from " + path);
            }
            else
            {
                setStatus(state, "Load failed");
            }
        }
        else if (ch == 'c')
        {
            int follower_count = expectedSeatCount(state);
            int selected_count = countSelected(state.grid);
            if (selected_count != follower_count)
            {
                setStatus(state, "Seat count must equal follower count");
                continue;
            }
            publishUAVSwarmCMD(swarm_pub, sunray_msgs::UAVSwarmCMD::SWARM_FORMATION,
                               sunray_msgs::UAVSwarmCMD::CUSTOM, 0.0f, 0.0, 0.0, 0.0, 0.0,
                               collectOffsets(state.grid));
            setStatus(state, "Custom formation sent");
        }
        else if (ch == 'm')
        {
            std::string input = promptLine("Leader goal x y z yaw: ");
            std::stringstream ss(input);
            double x = 0.0, y = 0.0, z = 0.0, yaw = 0.0;
            if (ss >> x >> y >> z >> yaw)
            {
                publishUAVSwarmCMD(swarm_pub, sunray_msgs::UAVSwarmCMD::SWARM_FORMATION,
                                   sunray_msgs::UAVSwarmCMD::KEEP_FORMATION, 0.0f, x, y, z, yaw);
                setStatus(state, "Leader goal sent");
            }
            else
            {
                setStatus(state, "Invalid goal input");
            }
        }
        else if (ch == '1')
        {
            publishUAVSwarmCMD(swarm_pub, sunray_msgs::UAVSwarmCMD::SWARM_FORMATION, sunray_msgs::UAVSwarmCMD::RING);
            setStatus(state, "Formation: ring");
        }
        else if (ch == '4')
        {
            publishUAVSwarmCMD(swarm_pub, sunray_msgs::UAVSwarmCMD::SWARM_FORMATION, sunray_msgs::UAVSwarmCMD::LINE);
            setStatus(state, "Formation: line");
        }
        else if (ch == '5')
        {
            publishUAVSwarmCMD(swarm_pub, sunray_msgs::UAVSwarmCMD::SWARM_FORMATION,
                               sunray_msgs::UAVSwarmCMD::COLUMN);
            setStatus(state, "Formation: column");
        }
        else if (ch == '6')
        {
            publishUAVSwarmCMD(swarm_pub, sunray_msgs::UAVSwarmCMD::SWARM_FORMATION,
                               sunray_msgs::UAVSwarmCMD::V_SHAPE);
            setStatus(state, "Formation: v_shape");
        }
        else if (ch == '7')
        {
            publishUAVSwarmCMD(swarm_pub, sunray_msgs::UAVSwarmCMD::SWARM_FORMATION, sunray_msgs::UAVSwarmCMD::WEDGE);
            setStatus(state, "Formation: wedge");
        }
        else if (ch == '2')
        {
            publishUAVSwarmCMD(swarm_pub, sunray_msgs::UAVSwarmCMD::SWARM_FORMATION, sunray_msgs::UAVSwarmCMD::EXPAND);
            setStatus(state, "Spacing: expand");
        }
        else if (ch == '3')
        {
            publishUAVSwarmCMD(swarm_pub, sunray_msgs::UAVSwarmCMD::SWARM_FORMATION,
                               sunray_msgs::UAVSwarmCMD::CONTRACT);
            setStatus(state, "Spacing: contract");
        }
        else if (ch == 't')
        {
            publishUAVSwarmCMD(swarm_pub, sunray_msgs::UAVSwarmCMD::SWARM_TAKEOFF);
            setStatus(state, "Takeoff sent");
        }
        else if (ch == 'g')
        {
            publishUAVSwarmCMD(swarm_pub, sunray_msgs::UAVSwarmCMD::SWARM_LAND);
            setStatus(state, "Land sent");
        }
        else if (ch == 'h')
        {
            publishUAVSwarmCMD(swarm_pub, sunray_msgs::UAVSwarmCMD::SWARM_HOVER);
            setStatus(state, "Hover sent");
        }
        else if (ch == 'b')
        {
            publishUAVSwarmCMD(swarm_pub, sunray_msgs::UAVSwarmCMD::SWARM_RETURN);
            setStatus(state, "Return home sent");
        }
        else if (ch == 'q')
        {
            break;
        }
    }

    endwin();
    return 0;
}
