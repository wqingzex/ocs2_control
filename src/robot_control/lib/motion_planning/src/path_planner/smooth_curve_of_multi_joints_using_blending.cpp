#include "motion_planning/path_planner/smooth_curve_of_multi_joints_using_blending.h"
#include "motion_planning/common/logger.h"
#include "motion_planning/path_planner/bezier_curve.h"
#include "motion_planning/path_planner/movej.h"
#include <fstream>
#include <rclcpp/logging.hpp>

namespace planning
{
    bool SmoothCurveOfMultiJointsUsingBlending::init(
        const TrajectoryInitParameters& traj_para)
    {
        period = traj_para.period;
        time_scaling_factor = 1.0;
        planners.clear();
        init_movej_planners.clear();
        if (traj_para.multiple_points.size() < 3)
        {
            RCLCPP_ERROR_STREAM(planning::logger(), "Input points less than 3,please use the correct planner!");
            return false;
        }
        if (traj_para.multiple_parameters.size() + 1 !=
            traj_para.multiple_points.size() &&
            !traj_para.use_common_parameters)
        {
            RCLCPP_ERROR_STREAM(planning::logger(), "The input points and parameters don't mathch,please enter "
                "matching parameters! ");
            return false;
        }
        if (!addMoveJ(traj_para))
        {
            return false;
        }
        if (!addAllPlanners())
        {
            RCLCPP_ERROR_STREAM(planning::logger(), "Init all planners error!");
            return false;
        }
        setVectorTotalTimes();
        current_planner = planners[0];
        current_planner_index = 0;
        time_scaling_factor = 1.0;
        run_completed = false;
        return true;
    }

    TrajectPoint
    SmoothCurveOfMultiJointsUsingBlending::calculatePointAtTInRealTime(double t)
    {
        double tmptime = real_start_time;
        bool find_interval_planner = false;
        for (size_t i = 0; i < nr_of_planners; i++)
        {
            tmptime += total_time_of_planners[i];
            if (t <= tmptime)
            {
                current_planner = planners[i];
                find_interval_planner = true;
                break;
            }
        }
        if (!find_interval_planner)
        {
            current_planner = planners[nr_of_planners];
        }
        return calculatePointAtTInCurve(t);
    }

    TrajectPoint
    SmoothCurveOfMultiJointsUsingBlending::calculatePointAtTInCurve(double t)
    {
        TrajectPoint res;
        res = current_planner->calculatePointAtTInRealTime(t);
        return res;
    }

    TrajectPoint SmoothCurveOfMultiJointsUsingBlending::run()
    {
        TrajectPoint res;
        current_real_time += period * time_scaling_factor;
        if (current_planner->isMotionOver())
        {
            current_planner_index++;
            if (current_planner_index < nr_of_planners)
            {
                current_planner = planners[current_planner_index];
            }
            else
            {
                run_completed = true;
            }
        }
        res = calculatePointAtTInCurve(current_real_time);
        if (current_planner_index >= nr_of_planners - 1)
        {
            run_completed = current_planner->run_completed;
        }
        //
        // //save_code
        // std::ofstream file(
        //           "/home/lina/lina/data/smooth_curve_of_joint.csv",std::ios::app);
        // file<<current_real_time<<" ,";
        // for (size_t i = 0; i < res.joint_pos.getJointSize(); ++i) {
        //     file << res.joint_pos(i);
        //     if (i < res.joint_pos.getJointSize()-1) {
        //         file << ",";
        //     }
        // }
        // file<<std::endl;
        // file.close();

        return res;
    }

    void SmoothCurveOfMultiJointsUsingBlending::getIthStartAndEndPoints(
        TrajectPoint& start_point, TrajectPoint& end_point,
        const TrajectoryInitParameters& traj_para, size_t i)
    {
        start_point = traj_para.multiple_points[i];
        end_point = traj_para.multiple_points[i + 1];
        // 保证轨迹的起点和终点没有转接
        if (i == 0)
        {
            start_point.blend_tolerance_ratio_of_movej = 0.0;
        }
        if (i == nr_of_init_movej - 1)
        {
            end_point.blend_tolerance_ratio_of_movej = 0.0;
        }
    }

    bool SmoothCurveOfMultiJointsUsingBlending::addMoveJ(
        const TrajectoryInitParameters& traj_para)
    {
        nr_of_joints = traj_para.multiple_points[0].joint_pos.getJointSize();
        nr_of_init_movej = traj_para.multiple_points.size() - 1;
        TrajectPoint current_start_point, current_end_point;
        if (traj_para.use_common_parameters)
        {
            TrajectoryParameter tmp_para = traj_para.parameter;
            tmp_para.time_mode = false;
            for (size_t i = 0; i < nr_of_init_movej; i++)
            {
                getIthStartAndEndPoints(current_start_point, current_end_point, traj_para,
                                        i);
                if (!addSingleMoveJ(current_start_point, current_end_point, tmp_para,
                                    traj_para.period))
                {
                    RCLCPP_ERROR_STREAM(planning::logger(), "Init movej error at " << i << "th movej trajectory!");
                    return false;
                }
            }
            if (traj_para.parameter.time_mode)
            {
                double total_time_of_movejs = 0.0, common_time_scaling = 1.0;
                for (size_t i = 0; i < nr_of_init_movej; i++)
                {
                    total_time_of_movejs += init_movej_planners[i]->getCurveTime();
                }
                if (traj_para.parameter.total_time > period)
                {
                    common_time_scaling =
                        total_time_of_movejs / traj_para.parameter.total_time;
                    for (size_t i = 0; i < nr_of_init_movej; i++)
                    {
                        init_movej_planners[i]->time_scaling_factor = common_time_scaling;
                    }
                }
            }
        }
        else
        {
            // 不使用公共参数，，由用户设定不同的参数
            for (size_t i = 0; i < nr_of_init_movej; i++)
            {
                getIthStartAndEndPoints(current_start_point, current_end_point, traj_para,
                                        i);
                if (!addSingleMoveJ(current_start_point, current_end_point,
                                    traj_para.multiple_parameters[i], traj_para.period))
                {
                    RCLCPP_ERROR_STREAM(planning::logger(), "Init movej error at " << i << "th movej trajectory!");
                    return false;
                }
            }
        }

        return true;
    }

    bool SmoothCurveOfMultiJointsUsingBlending::addSingleMoveJ(
        const TrajectPoint& current_start_point,
        const TrajectPoint& current_end_point, const TrajectoryParameter& para,
        double period)
    {
        auto movej_ptr = std::make_shared<moveJ>();

        TrajectoryInitParameters current_movej_para(current_start_point,
                                                    current_end_point, para, period);

        if (!movej_ptr->init(current_movej_para))
        {
            return false;
        }
        init_movej_planners.push_back(movej_ptr);
        return true;
    };

    bool SmoothCurveOfMultiJointsUsingBlending::addAllPlanners()
    {
        // 设置贝塞尔曲线转接起点，终点
        TrajectPoint bezier_start_point(nr_of_joints);
        double start_time_without_transfer;
        TrajectPoint bezier_middle_point(nr_of_joints);
        TrajectPoint bezier_end_point(nr_of_joints);
        double end_time_without_transfer;
        TrajectoryParameter bezier_para;
        TrajectoryInitParameters bezier_init_para(nr_of_joints);
        BlendMessage current_movej_blend_message;
        BlendMessage previous_movej_blend_message;
        bool previous_trajectory_stopped = true;
        bool let_two_sides_time_equal = true;
        int planners_index = -1;
        // 从第1个开始到n个movej
        for (size_t i = 0; i < nr_of_init_movej; i++)
        {
            current_movej_blend_message = init_movej_planners[i]->get_blend_message();
            if (previous_trajectory_stopped)
            {
                // 如果之前的轨迹停下来，前面没有转接
                if (current_movej_blend_message
                    .end_point_setting_blend_point)
                {
                    // 轨迹末有转接
                    previous_trajectory_stopped = false;
                    bezier_start_point = init_movej_planners[i]->calculatePointAtTInCurve(
                        current_movej_blend_message.blend_end_time);
                    start_time_without_transfer =
                        (init_movej_planners[i]->getCurveTime() - bezier_start_point.time) /
                        init_movej_planners[i]->time_scaling_factor;
                    init_movej_planners[i]->setCurveStopTime(bezier_start_point.time);
                    init_movej_planners[i]->updateTotalTime();
                    planners.push_back(init_movej_planners[i]);
                    planners_index++;
                    planners[planners_index]->run_completed = false;
                }
                else
                {
                    // 当前轨迹不转接，直接将当前轨迹存到vector里面
                    planners.push_back(init_movej_planners[i]);
                    planners_index++;
                    planners[planners_index]->run_completed = false;
                    previous_trajectory_stopped = true;
                }
            }
            else
            {
                // 前一个轨迹的末端有转接，那么这个轨迹也有转接，首先要算一下贝塞尔转接终点
                bezier_middle_point.joint_pos = init_movej_planners[i]->getStartPoint();
                bezier_end_point = init_movej_planners[i]->calculatePointAtTInCurve(
                    current_movej_blend_message.blend_start_time);
                init_movej_planners[i]->setCurveStartTime(
                    bezier_end_point.time); // 设置下一个movej的起点
                end_time_without_transfer =
                    bezier_end_point.time / init_movej_planners[i]->time_scaling_factor;

                // 为了避免转接两侧的时间差别较大可能带来的转接轨迹较大的速度和加速度，这里增加一个让两侧时间相等的逻辑
                if (let_two_sides_time_equal)
                {
                    if (end_time_without_transfer <
                        start_time_without_transfer)
                    {
                        // 终点侧的时间比较小，这个时候改小上一个的转接点
                        start_time_without_transfer = end_time_without_transfer;
                        // 求出对应的curve time
                        double trans_start_curve_time =
                            init_movej_planners[i - 1]->getCurveTime() -
                            start_time_without_transfer *
                            init_movej_planners[i - 1]->time_scaling_factor;
                        // 重新计算上一个转接点的信息

                        if (previous_movej_blend_message
                            .blend_point_met)
                        {
                            // 如果上一个轨迹的中间相遇了，没有在planner里面添加，需要添加一下
                            bezier_start_point =
                                init_movej_planners[i - 1]->calculatePointAtTInCurve(
                                    trans_start_curve_time);

                            init_movej_planners[i - 1]->setCurveStopTime(
                                bezier_start_point.time);
                            init_movej_planners[i - 1]->updateTotalTime();
                            planners.push_back(init_movej_planners[i - 1]);
                            planners_index++;
                            planners[planners_index]->run_completed = false;
                        }
                        else
                        {
                            // 如果上一个轨迹已经添加到planner里面了，就直接在上一个里面修改
                            bezier_start_point =
                                planners[planners_index]->calculatePointAtTInCurve(
                                    trans_start_curve_time);
                            planners[planners_index]->setCurveStopTime(bezier_start_point.time);
                            planners[planners_index]->updateTotalTime();
                        }
                    }
                    else
                    {
                        // 终点侧的时间比较大，可以直接把这一侧的时间改小
                        end_time_without_transfer = start_time_without_transfer;
                        double trans_end_curve_time =
                            end_time_without_transfer *
                            init_movej_planners[i]->time_scaling_factor;
                        bezier_end_point = init_movej_planners[i]->calculatePointAtTInCurve(
                            trans_end_curve_time);
                        init_movej_planners[i]->setCurveStartTime(
                            bezier_end_point.time); // 重新设置下一个movej的起点
                    }
                }

                bezier_para.time_mode = true;
                bezier_para.total_time =
                    start_time_without_transfer + end_time_without_transfer;
                bezier_init_para.start_point = bezier_start_point;
                bezier_init_para.middle_point = bezier_middle_point;
                bezier_init_para.has_middle_point = true;
                bezier_init_para.end_point = bezier_end_point;
                bezier_init_para.parameter = bezier_para;
                auto bezier_ptr = std::make_shared<BezierCurve>();
                if (!bezier_ptr->init(bezier_init_para))
                {
                    RCLCPP_ERROR_STREAM(planning::logger(), "Init bezier error at " << i << "th movej trajectory!");
                    return false;
                }
                planners.push_back(bezier_ptr);
                planners_index++;
                planners[planners_index]->run_completed = false;

                // 转接后的轨迹添加：
                if (current_movej_blend_message
                    .end_point_setting_blend_point)
                {
                    // 轨迹末有转接
                    previous_trajectory_stopped = false;
                    if (!current_movej_blend_message.blend_point_met)
                    {
                        bezier_start_point = init_movej_planners[i]->calculatePointAtTInCurve(
                            current_movej_blend_message.blend_end_time);
                        start_time_without_transfer =
                            (init_movej_planners[i]->getCurveTime() -
                                bezier_start_point.time) /
                            init_movej_planners[i]->time_scaling_factor;
                        init_movej_planners[i]->setCurveStopTime(bezier_start_point.time);
                        init_movej_planners[i]->updateTotalTime();
                        planners.push_back(init_movej_planners[i]);
                        planners_index++;
                        planners[planners_index]->run_completed = false;
                    }
                    else
                    {
                        // 两个转接点相遇加就不把movej添加进来，只是更新下一个转接的起点
                        bezier_start_point = bezier_end_point;
                        start_time_without_transfer =
                            (init_movej_planners[i]->getCurveTime() -
                                bezier_start_point.time) /
                            init_movej_planners[i]->time_scaling_factor;
                    }
                }
                else
                {
                    // 当前轨迹不转接，直接将当前轨迹存到vector里面
                    init_movej_planners[i]->updateTotalTime();
                    planners.push_back(init_movej_planners[i]);
                    planners_index++;
                    planners[planners_index]->run_completed = false;
                    previous_trajectory_stopped = true;
                }
            }
            previous_movej_blend_message = current_movej_blend_message;
        }

        return true;
    };

    void SmoothCurveOfMultiJointsUsingBlending::setVectorTotalTimes()
    {
        double current_planner_totaltime = 0.0;
        total_time = 0.0;
        nr_of_planners = planners.size();
        for (size_t i = 0; i < nr_of_planners; i++)
        {
            current_planner_totaltime = planners[i]->getTotalTime();
            total_time_of_planners.push_back(current_planner_totaltime);
            if (i > 0)
            {
                planners[i]->setRealStartTime(total_time + real_start_time);
            }

            total_time += current_planner_totaltime;
        }
    };
} // namespace planning
