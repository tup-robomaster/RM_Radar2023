#include "../include/MapMapping.h"

MapMapping::MapMapping()
{
    vector<MapLocation3D> temp_1(10, MapLocation3D());
    this->_location3D.swap(temp_1);
    vector<vector<MapLocation3D>> temp_2(2, vector<MapLocation3D>(10, MapLocation3D()));
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
        bool do_pre = this->_location3D[i].x != 0 && this->_location3D[i].y != 0 && (this->_location_cache[0][i].x != 0 || this->_location_cache[0][i].y != 0) && (this->_location_cache[i][1].x != 0 || this->_location_cache[1][i].y != 0) && this->_location_pred_time[i] != 1;
        if (do_pre)
        {
            float m_v[2] = {Pre_radio * (this->_location_cache[1][i].x - this->_location_cache[1][i].x), Pre_radio * (this->_location_cache[1][i].y - this->_location_cache[0][i].y)};
            this->_location3D[i].x = m_v[0] + this->_location_cache[1][i].x;
            this->_location3D[i].y = m_v[0] + this->_location_cache[1][i].y;
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

vector<ArmorBoundingBox> MapMapping::_IoU_prediction()
{
    map<int, int>::iterator iter;
        iter = this->_ids.begin();
    while (iter != this->_ids.end())
    {
        
    }
}

void MapMapping::push_T(Mat &rvec_input, Mat &tvec_input)
{
    rvec_input.copyTo(this->rvec);
    tvec_input.copyTo(this->tvec);
    Mat rvec_Matrix;
    Rodrigues(this->rvec, rvec_Matrix);
    Mat T_Matrix = Mat::zeros(Size(4, 4), CV_16F);
    rvec_Matrix.copyTo(T_Matrix(Rect(0, 0, 3, 3)));
    T_Matrix.at<float>(Point2i(3, 0)) = this->tvec.at<float>(Point2i(0, 0));
    T_Matrix.at<float>(Point2i(3, 1)) = this->tvec.at<float>(Point2i(1, 0));
    T_Matrix.at<float>(Point2i(3, 2)) = this->tvec.at<float>(Point2i(2, 0));
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

void MapMapping::mergeUpdata(vector<ArmorBoundingBox> &tensorRTbbox, vector<ArmorBoundingBox> &Ioubbox)
{
    if (!this->_pass_flag)
    {
        fmt::print(fg(fmt::color::red) | fmt::emphasis::bold,
                   "[ERROR], Can't get _T !\n");
        return;
    }
    vector<MapLocation3D> temp(10, MapLocation3D());
    this->_location3D.swap(temp);
    vector<ArmorBoundingBox> locations;
    if (tensorRTbbox.size() > 0)
    {
        for (size_t i = 0; i < tensorRTbbox.size(); ++i)
        {
            if (tensorRTbbox[i].depth != 0)
            {
                tensorRTbbox[i].flag = true;
                locations.emplace_back(tensorRTbbox[i]);
            }
            else
                tensorRTbbox[i].flag = false;
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
            bool TRTtype = false; // 神经网络预测标志位
            for (const auto &it : locations)
            {
                if ((int)it.cls == key)
                {
                    Matrix<float, 4, 1> xyzu;
                    xyzu << (it.x0 + it.w / 2) * it.depth, (it.y0 + it.h / 2) * it.depth, it.depth, 1.f;
                    Matrix<float, 4, 1> dst_xyzu;
                    dst_xyzu << this->_T * xyzu;
                    al.id = it.cls;
                    al.x = dst_xyzu(0, 0);
                    al.y = dst_xyzu(1, 0);
                    al.z = dst_xyzu(2, 0);
                    al.flag = true;
                    TRTtype = true;
                    if (Z_A)
                        this->adjust_z_one(al);
                    break;
                }
                // TODO:添加IOU预测
            }
            if (Z_A)
            {
                if (al.flag)
                    cache_pred.emplace_back(al);
            }
            if (!TRTtype)
            {
                // TODO:添加IOU预测
            }
            else
            {
                // TODO:进阶预测控制
                pred_loc.emplace_back(al);
            }
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
        }
        if (L_P)
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