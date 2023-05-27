#include "../include/MapMapping.h"

MapMapping::MapMapping()
{
    MapLocation3D temp;
    vector<MapLocation3D> temp_1(this->_ids.size(), temp);
    this->_location3D.swap(temp_1);
    vector<vector<MapLocation3D>> temp_2(2, vector<MapLocation3D>(this->_ids.size(), temp));
    this->_location_cache.swap(temp_2);
}

MapMapping::~MapMapping()
{
}

bool MapMapping::_is_pass()
{
    return this->_pass_flag;
}

void MapMapping::_location_prediction()
{
    for (size_t i = 0; i < this->_location3D.size(); ++i)
    {
        bool do_pre = this->_location3D[i].x != 0 && this->_location3D[i].y != 0 && this->_location_cache[0][i].x != 0 && this->_location_cache[0][i].y != 0 && this->_location_cache[1][i].x != 0 && this->_location_cache[1][i].y != 0 && this->_location_pred_time[i] != 1;
        if (do_pre)
        {
            float m_v[2] = {Pre_radio * (this->_location_cache[1][i].x - this->_location_cache[0][i].x), Pre_radio * (this->_location_cache[1][i].y - this->_location_cache[0][i].y)};
            this->_location3D[i].x = m_v[0] + this->_location_cache[1][i].x;
            this->_location3D[i].y = m_v[1] + this->_location_cache[1][i].y;
        }
        if (this->_location3D[i].x != 0 && this->_location3D[i].y != 0 && this->_location_pred_time[i] == 1)
            this->_location_pred_time[i] = 0;
        if (do_pre && this->_location_pred_time[i] == 0)
            this->_location_pred_time[i] = Pre_Time + 1;
        if (do_pre)
            --this->_location_pred_time[i];
        this->_location_cache[0] = this->_location_cache[1];
        this->_location_cache[1] = this->_location3D;
    }
}

void MapMapping::_plot_region_rect(vector<Point3f> &points, Mat &frame, Mat &K_0, Mat &C_0)
{
    vector<Point2f> ips_pre;
    vector<Point2i> ips_dst;
    cv::projectPoints(points, this->rvec, this->tvec, K_0, C_0, ips_pre);
    cv::Mat(ips_pre).convertTo(ips_dst, CV_32SC1);
    circle(frame, ips_dst[0], 5, Scalar(0, 255, 0), 2);
    circle(frame, ips_dst[1], 5, Scalar(0, 255, 255), 2);
    circle(frame, ips_dst[2], 5, Scalar(255, 255, 0), 2);
    circle(frame, ips_dst[3], 5, Scalar(255, 0, 0), 2);
    line(frame, ips_dst[0], ips_dst[1], Scalar(0, 255, 0), 3);
    line(frame, ips_dst[1], ips_dst[2], Scalar(0, 255, 0), 3);
    line(frame, ips_dst[2], ips_dst[3], Scalar(0, 255, 0), 3);
    line(frame, ips_dst[3], ips_dst[0], Scalar(0, 255, 0), 3);
}

vector<ArmorBoundingBox> MapMapping::_IoU_prediction(vector<bboxAndRect> pred, vector<Rect> sepboxs)
{
    vector<ArmorBoundingBox> pred_bbox;
    map<int, int>::iterator iter;
    iter = this->_ids.begin();
    if (this->_IoU_pred_cache.size() > 0)
    {
        bboxAndRect cached_pred;
        while (iter != this->_ids.end())
        {
            bool cache_check = false;
            bool pred_check = false;
            for (const auto &it : this->_IoU_pred_cache)
            {
                cache_check = it.armor.cls == iter->first ? true : false;
                if (cache_check)
                    cached_pred.armor = it.armor;
            }
            for (const auto &it : pred)
            {
                pred_check = it.armor.cls != iter->first ? true : false;
            }
            if (cache_check && pred_check)
            {
                vector<float> iou;
                for (const auto &it : sepboxs)
                {
                    float x1 = f_max(cached_pred.armor.x0, it.x);
                    float x2 = f_min(cached_pred.armor.x0 + cached_pred.armor.w, it.x + it.width);
                    float y1 = f_max(cached_pred.armor.y0, it.y);
                    float y2 = f_min(cached_pred.armor.y0 + cached_pred.armor.h, it.y + it.height);
                    float overlap = f_max(0, x2 - x1) * f_max(0, y2 - y1);
                    float area = cached_pred.armor.w * cached_pred.armor.h;
                    iou.emplace_back(overlap / area);
                }
                int max_index = max_element(iou.begin(), iou.end()) - iou.begin();
                if (iou[max_index] > IoU_THRE)
                {
                    Rect current_rect = sepboxs[max_index];
                    current_rect.width = floor(current_rect.width / 3);
                    current_rect.height = floor(current_rect.height / 5);
                    current_rect.x += current_rect.width;
                    current_rect.y += current_rect.height * 3;
                    pred_bbox.emplace_back(ArmorBoundingBox{true, (float)current_rect.x, (float)current_rect.y, (float)current_rect.width, (float)current_rect.height, pred[max_index].armor.cls});
                }
            }
            iter++;
        }
    }
    if (pred.size() > 0)
        this->_IoU_pred_cache.swap(pred);
    else
        vector<bboxAndRect>().swap(this->_IoU_pred_cache);
    return pred_bbox;
}

void MapMapping::push_T(Mat &rvec_input, Mat &tvec_input)
{
    rvec_input.copyTo(this->rvec);
    tvec_input.copyTo(this->tvec);

    stringstream ss_rvec, ss_tvec;
    ss_rvec << "rvec= " << endl
            << " " << rvec_input << endl
            << endl;
    ss_tvec << "tvec= " << endl
            << " " << tvec_input << endl
            << endl;

    this->logger->info(ss_rvec.str());
    this->logger->info(ss_tvec.str());
    Mat rvec_Matrix;
    Rodrigues(this->rvec, rvec_Matrix);
    Mat T_Matrix = Mat::zeros(Size(4, 4), CV_32F);
    rvec_Matrix.copyTo(T_Matrix(Rect(0, 0, 3, 3)));
    this->tvec.copyTo(T_Matrix(Rect(3, 0, 1, 3)));
    T_Matrix.at<_Float32>(Point2i(3, 3)) = 1.f;
    cv2eigen(T_Matrix, this->_T);
    this->_T << this->_T.inverse();
    Matrix<float, 4, 1> m1;
    m1 << 0.f, 0.f, 0.f, 1.f;
    this->cameraPostion << (this->_T * m1).topRows(3);
    this->_pass_flag = true;
}

vector<MapLocation3D> MapMapping::getloc()
{
    return this->_location3D;
}

void MapMapping::mergeUpdata(vector<bboxAndRect> &pred, vector<ArmorBoundingBox> &Ioubbox, Mat &K_0, Mat &C_0)
{
    if (!this->_pass_flag)
    {
        this->logger->error("Can't get _T !");
        return;
    }
    vector<MapLocation3D> temp(this->_ids.size());
    this->_location3D.swap(temp);
    vector<ArmorBoundingBox> locations;
    if (pred.size() > 0)
    {
        for (size_t i = 0; i < pred.size(); ++i)
        {
            if (pred[i].armor.depth != 0 && !isnan(pred[i].armor.depth))
            {
                pred[i].armor.flag = true;
                locations.emplace_back(pred[i].armor);
            }
            else
                pred[i].armor.flag = false;
        }
    }
    if (Ioubbox.size() > 0)
    {
        for (size_t i = 0; i < Ioubbox.size(); ++i)
        {
            int item_cls = int(Ioubbox[i].cls);
            bool check = std::find_if(locations.begin(), locations.end(), [item_cls](ArmorBoundingBox item)
                                      { return int(item.cls) == item_cls; }) == locations.end();
            if (!check)
                continue;
            if (Ioubbox[i].depth != 0 && !isnan(Ioubbox[i].depth))
            {
                Ioubbox[i].flag = true;
                locations.emplace_back(Ioubbox[i]);
            }
            else
                Ioubbox[i].flag = false;
        }
    }
    if (locations.size() > 0)
    {
        vector<MapLocation3D> pred_loc;
        vector<MapLocation3D> cache_pred;
        map<int, int>::iterator iter;
        iter = this->_ids.begin();
        while (iter != this->_ids.end())
        {
            int key = iter->first;
            MapLocation3D al;
            for (const auto &it : locations)
            {
                if ((int)it.cls == key)
                {
                    Matrix<float, 4, 1> xyzu;
                    vector<Point2f> center = {Point2f((it.x0 + it.w / 2.f), (it.y0 + it.h / 2.f))};
                    cv::undistortPoints(center, center, K_0, C_0);
                    xyzu << center[0].x * it.depth, center[0].y * it.depth, it.depth, 1.f;
                    Matrix<float, 4, 1> dst_xyzu;
                    dst_xyzu << this->_T * xyzu;
                    al.id = it.cls;
                    al.x = dst_xyzu(0, 0);
                    al.y = dst_xyzu(1, 0);
                    al.z = dst_xyzu(2, 0);
                    al.flag = true;
                    if (Z_A)
                        this->adjust_z_one(al);
                    break;
                }
            }
            if (Z_A)
            {
                if (al.flag)
                    cache_pred.emplace_back(al);
            }
            if(al.flag)
                pred_loc.emplace_back(al);
            ++iter;
        }
        if (Z_A)
        {
            if (pred_loc.size() > 0)
            {
                this->cached_location3D.swap(pred_loc);
            }
        }
        for (size_t i = 0; i < pred_loc.size(); ++i)
        {
            pred_loc[i].z += Real_Size_W;
            this->_location3D[this->_ids[(int)pred_loc[i].id]] = pred_loc[i];
            this->logger->info("LOC: [CLS] " + to_string(pred_loc[i].id) + " [x] " + to_string(pred_loc[i].x) + " [y] " + to_string(pred_loc[i].y) + " [z] " + to_string(pred_loc[i].z));
        }
        if (L_P)
            // TODO: FIX HERE
            this->_location_prediction();
    }
}
// TODO: 待验证
void MapMapping::adjust_z_one(MapLocation3D &loc)
{
    MapLocation3D pre_loc;
    for (const auto &it : this->cached_location3D)
    {
        if (it.id == loc.id)
            pre_loc = it;
    }
    if (!pre_loc.flag)
        return;
    if (loc.z - pre_loc.z > Z_THRE)
    {
        Matrix<float, 3, 1> line;
        line << loc.x - this->cameraPostion(0, 0), loc.y - this->cameraPostion(1, 0), loc.z - this->cameraPostion(2, 0);
        float radio = (pre_loc.z - this->cameraPostion(2, 0)) / line(2, 0);
        loc.x = radio * line(0, 0) + this->cameraPostion(0, 0);
        loc.y = radio * line(1, 0) + this->cameraPostion(1, 0);
        loc.z = radio * line(2, 0) + this->cameraPostion(2, 0);
    }
}