#include "../include/location.h"

static vector<Point2f> pick_points;
static FrameBag frame;
static bool flag = false;

Location::Location()
{
    this->location_targets["red_base"] = Point3f(1.760, -15. + 7.539, 0.200 + 0.920);
    this->location_targets["blue_outpost"] = Point3f(16.776, -15. + 12.565, 1.760);
    this->location_targets["red_outpost"] = Point3f(11.176, -15. + 2.435, 1.760);
    this->location_targets["blue_base"] = Point3f(26.162, -15. + 7.539, 0.200 + 0.920);
    this->location_targets["r_rt"] = Point3f(8.805, -5.728 - 0.660, 0.120 + 0.495);
    this->location_targets["r_lt"] = Point3f(8.805, -5.728, 0.120 + 0.495);
    this->location_targets["b_rt"] = Point3f(19.200, -9.272 + 0.660, 0.120 + 0.495);
    this->location_targets["b_lt"] = Point3f(19.200, -9.272, 0.120 + 0.495);
    vector<Point2f>().swap(pick_points);
    frame = FrameBag();
}

Location::~Location()
{
}

void __callback__click(int event, int x, int y, int flage, void *param)
{
    Mat img_cut = Mat::zeros(Size(200, 200), CV_8UC3);
    cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 30, 0.001);
    Rect rect;
    switch (event)
    {
    case MouseEventTypes::EVENT_MOUSEMOVE:
        rect = getWindowImageRect("PickPoints");
        frame.frame(Rect(Point(max(x - 100, 0), max(y - 100, 0)), Point(min(x + 100, frame.frame.cols - 1), min(y + 100, frame.frame.rows - 1)))).copyTo(img_cut(Rect(0, 0, min(x + 100, frame.frame.cols - 1) - max(x - 100, 0), min(y + 100, frame.frame.rows - 1) - max(y - 100, 0))));
        circle(img_cut, Point(100, 100), 1, Scalar(0, 255, 0), 1);
        imshow("ZOOM_WINDOW", img_cut);
        moveWindow("ZOOM_WINDOW", rect.width - 400, rect.height + 200);
        resizeWindow("ZOOM_WINDOW", 400, 400);
        break;
    case MouseEventTypes::EVENT_LBUTTONDOWN:
        if (!flag)
        {
            flag = true;
            fmt::print(fg(fmt::color::aqua) | fmt::emphasis::bold,
                       "[INFO], Pick {}|{}!\n", x, y);
            vector<Point2f> temp_corner;
            temp_corner.emplace_back(Point2f(x, y));
            Mat grey;
            cvtColor(frame.frame, grey, COLOR_BGR2GRAY);
            cornerSubPix(grey, temp_corner, cv::Size(5, 5), cv::Size(-1, -1), criteria);
            pick_points.emplace_back(temp_corner[0]);
            circle(frame.frame, Point(x, y), 2, Scalar(0, 255, 0), 1);
        }
        break;
    }
}

bool Location::locate_pick(CameraThread &cap, int enemy, Mat &rvec_Mat, Mat &tvec_Mat)
{
    Mat K_0;
    Mat C_0;
    Mat E_0;
    if (!read_yaml(K_0, C_0, E_0))
        return false;
    Point3f red_base = this->location_targets["red_base"];
    Point3f blue_outpost = this->location_targets["blue_outpost"];
    Point3f red_outpost = this->location_targets["red_outpost"];
    Point3f blue_base = this->location_targets["blue_base"];
    Point3f r_rt = this->location_targets["r_rt"];
    Point3f r_lt = this->location_targets["r_lt"];
    Point3f b_rt = this->location_targets["b_rt"];
    Point3f b_lt = this->location_targets["b_lt"];
    map<int, vector<string>> tips{{0, {"red_base", "blue_outpost", "b_right_top", "red_outpost"}}, {1, {"blue_base", "red_outpost", "r_right_top", "blue_outpost"}}};
    vector<Point3f> ops;
    if (enemy == 0)
    {
        ops.emplace_back(red_base);
        ops.emplace_back(blue_outpost);
        ops.emplace_back(b_rt);
        ops.emplace_back(red_outpost);
    }
    else
    {
        ops.emplace_back(blue_base);
        ops.emplace_back(red_outpost);
        ops.emplace_back(r_rt);
        ops.emplace_back(blue_outpost);
    }
    frame = cap.read();
    if (!cap.is_open() || !frame.flag)
        return false;
    int tip_w = floor(frame.frame.cols / 2);
    int tip_h = frame.frame.rows - 200;
    namedWindow("PickPoints", WindowFlags::WINDOW_NORMAL);
    resizeWindow("PickPoints", Size(1280, 780));
    setWindowProperty("PickPoints", WindowPropertyFlags::WND_PROP_TOPMOST, 1);
    moveWindow("PickPoints", 500, 300);
    namedWindow("ZOOM_WINDOW", WindowFlags::WINDOW_NORMAL);
    resizeWindow("ZOOM_WINDOW", 400, 400);
    setWindowProperty("ZOOM_WINDOW", WindowPropertyFlags::WND_PROP_TOPMOST, 1);
    setMouseCallback("PickPoints", __callback__click);
    while (true)
    {
        putText(frame.frame, tips[(int)(enemy)][pick_points.size()], Point(tip_w, tip_h), HersheyFonts::FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 255, 0), 2);
        for (const auto &it : pick_points)
            circle(frame.frame, it, 1, Scalar(0, 255, 0), 2);
        for (size_t i = 1; i < pick_points.size(); ++i)
            line(frame.frame, pick_points[i - 1], pick_points[i], Scalar(0, 255, 0), 2);
        imshow("PickPoints", frame.frame);
        if (flag)
        {
            if (pick_points.size() == 4)
            {
                line(frame.frame, pick_points[3], pick_points[0], Scalar(0, 255, 0), 2);
                imshow("PickPoints", frame.frame);
            }
            int key = waitKey(0);
            if (key == 90 || key == 122)
            {
                if (pick_points.size() == 4)
                    line(frame.frame, pick_points[3], pick_points[0], Scalar(0, 0, 255), 2);
                else if (pick_points.size() > 1)
                    line(frame.frame, pick_points[pick_points.size() - 1], pick_points[pick_points.size() - 2], Scalar(0, 0, 255), 2);
                circle(frame.frame, pick_points[pick_points.size() - 1], 1, Scalar(0, 0, 255), 2);
                pick_points.pop_back();
                imshow("PickPoints", frame.frame);
            }
            else if (key == 81 || key == 113)
            {
                destroyWindow("PickPoints");
                destroyWindow("ZOOM_WINDOW");
                return false;
            }
            flag = false;
        }
        else
        {
            waitKey(1);
        }
        if (pick_points.size() == 4)
            break;
        frame = cap.read();
        if (!cap.is_open() || !frame.flag)
        {
            destroyWindow("PickPoints");
            destroyWindow("ZOOM_WINDOW");
            return false;
        }  
    }
    destroyWindow("PickPoints");
    destroyWindow("ZOOM_WINDOW");
    if (!solvePnP(ops, pick_points, K_0, C_0, rvec_Mat, tvec_Mat, false, SolvePnPMethod::SOLVEPNP_P3P))
    {
        fmt::print(fg(fmt::color::red) | fmt::emphasis::bold,
                   "[ERROR], {}!\n", "PnP failed");
        return false;
    }
    return true;
}