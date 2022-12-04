#include "../PublicInclude/MapMapping.h"

MapMapping::MapMapping()
{
    vector<MapLocation3D> temp(10, MapLocation3D());
    this->_location3D.swap(temp);
    this->_ids[1] = 6;
    this->_ids[2] = 7;
    this->_ids[3] = 8;
    this->_ids[4] = 9;
    this->_ids[5] = 10;
    this->_ids[8] = 1;
    this->_ids[9] = 2;
    this->_ids[10] = 3;
    this->_ids[11] = 4;
    this->_ids[12] = 5;
}

MapMapping::~MapMapping()
{
}

bool MapMapping::_is_pass()
{
    return this->_pass_flag;
}

void MapMapping::push_T(Mat revc, Mat tevc)
{
    this->revc = revc;
    this->tvec = tvec;
    Mat revc_Matrix;
    Rodrigues(revc, revc_Matrix);
    Mat T_Matrix = Mat::zeros(Size(4, 4), CV_16F);
    revc_Matrix.copyTo(T_Matrix(Rect(0, 0, 3, 3)));
    T_Matrix.at<float>(Point2i(3, 0)) = tvec.at<float>(Point2i(0, 0));
    T_Matrix.at<float>(Point2i(3, 1)) = tvec.at<float>(Point2i(1, 0));
    T_Matrix.at<float>(Point2i(3, 2)) = tvec.at<float>(Point2i(2, 0));
    cv2eigen(T_Matrix, this->_T);
    this->_T = this->_T.inverse();
    this->_pass_flag = true;
}

vector<MapLocation3D> MapMapping::updata()
{
    return this->_location3D;
}

void MapMapping::mergeUpdata(vector<ArmorBoundingBox> &tensorRTbbox, vector<ArmorBoundingBox> &Ioubbox)
{
    if (!this->_pass_flag)
    {
        cout << "[ERROR]"
             << "Can't get _T !" << endl;
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
            bool TRTtype = false; //神经网络预测标志位
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
                    if(Z_A)
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
        if(Z_A)
        {
            if(pred_loc.size() > 0)
            {
                this->cached_location3D.swap(pred_loc);
            }
        }
        for (size_t i = 0; i < pred_loc.size(); ++i)
        {
            pred_loc[i].z += Real_Size_W;
            this->_location3D[this->_ids[(int)pred_loc[i].id]] = pred_loc[i];
        }
    }
}

void MapMapping::adjust_z_one(MapLocation3D &locs)
{

}