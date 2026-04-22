use crate::ros2::cmd::{spawn_fyt_cmd_listener, FytCommandState};
use crate::ros2::capture::{CaptureConfig, RosCaptureContext, RosCapturePlugin};
use crate::ros2::topic::*;
use crate::{
    Cooldown, GameLayer, InfantryChassis, InfantryGimbal, InfantryLaunchOffset, InfantryRoot,
    InfantryViewOffset, LocalInfantry, ProjectileSetting, arc_mutex, publisher,
    robomaster::power_rune::{PowerRune, Projectile, RuneIndex},
};
use avian3d::prelude::*;
use bevy::prelude::*;
use bevy::render::render_resource::TextureFormat;
use r2r::ClockType::SystemTime;
use r2r::geometry_msgs::msg::{Pose, PoseStamped};
use r2r::{Clock, Context, Node, std_msgs::msg::Header, tf2_msgs::msg::TFMessage};
use std::f32::consts::PI;
use std::time::Duration;
use std::{
    sync::{
        Arc, Mutex,
        atomic::{AtomicBool, Ordering},
    },
    thread::{self, JoinHandle},
};

pub const M_ALIGN_MAT3: Mat3 = Mat3::from_cols(
    Vec3::new(0.0, -1.0, 0.0), // M[0,0], M[1,0], M[2,0]
    Vec3::new(0.0, 0.0, 1.0),  // M[0,1], M[1,1], M[2,1]
    Vec3::new(-1.0, 0.0, 0.0), // M[0,2], M[1,2], M[2,2]
);

#[inline]
pub fn transform(bevy_transform: Transform) -> r2r::geometry_msgs::msg::Transform {
    let align_rot_mat = M_ALIGN_MAT3;
    let align_quat = Quat::from_mat3(&align_rot_mat);
    let new_rotation = align_quat * bevy_transform.rotation * align_quat.inverse();
    let new_translation = align_rot_mat * bevy_transform.translation;
    r2r::geometry_msgs::msg::Transform {
        translation: r2r::geometry_msgs::msg::Vector3 {
            x: new_translation.x as f64,
            y: new_translation.y as f64,
            z: new_translation.z as f64,
        },
        rotation: r2r::geometry_msgs::msg::Quaternion {
            x: new_rotation.x as f64,
            y: new_rotation.y as f64,
            z: new_rotation.z as f64,
            w: new_rotation.w as f64,
        },
    }
}

macro_rules! res_unwrap {
    ($res:tt) => {
        $res.0.lock().unwrap()
    };
}

#[derive(Resource)]
struct StopSignal(Arc<AtomicBool>);

#[derive(Resource)]
struct SpinThreadHandle(Option<JoinHandle<()>>);

#[derive(Component)]
pub struct MainCamera;

#[derive(Resource)]
pub struct RoboMasterClock(pub Arc<Mutex<Clock>>);

#[macro_export]
macro_rules! add_tf_frame {
    ($ls:ident, $hdr:expr, $id:expr, $translation:expr, $rotation:expr) => {
        $ls.push(::r2r::geometry_msgs::msg::TransformStamped {
            header: $hdr.clone(),
            child_frame_id: $id.to_string(),
            transform: crate::ros2::plugin::transform(
                Transform::IDENTITY
                    .with_translation($translation)
                    .with_rotation($rotation),
            ),
        });
    };
    ($ls:ident, $hdr:expr, $id:expr, $transform:expr) => {
        $ls.push(::r2r::geometry_msgs::msg::TransformStamped {
            header: $hdr.clone(),
            child_frame_id: $id.to_string(),
            transform: crate::ros2::plugin::transform($transform),
        });
    };
}

#[macro_export]
macro_rules! pose {
    ($hdr:expr) => {
        PoseStamped {
            header: $hdr.clone(),
            pose: Pose {
                position: r2r::geometry_msgs::msg::Point {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
                orientation: r2r::geometry_msgs::msg::Quaternion {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    w: 1.0,
                },
            },
        }
    };
}

fn capture_rune(
    camera: Single<&GlobalTransform, With<MainCamera>>,
    gimbal: Single<&GlobalTransform, (With<LocalInfantry>, With<InfantryGimbal>)>,
    _view_offset: Single<&InfantryViewOffset, With<LocalInfantry>>,

    runes: Query<(&GlobalTransform, &PowerRune)>,
    targets: Query<(&GlobalTransform, &RuneIndex, &Name)>,

    clock: ResMut<RoboMasterClock>,
    tf_publisher: ResMut<TopicPublisher<GlobalTransformTopic>>,
    gimbal_pose_pub: ResMut<TopicPublisher<GimbalPoseTopic>>,
    odom_pose_pub: ResMut<TopicPublisher<OdomPoseTopic>>,
    camera_pose_pub: ResMut<TopicPublisher<CameraPoseTopic>>,
) {
    let cam_transform = camera.into_inner();
    let stamp = Clock::to_builtin_time(&res_unwrap!(clock).get_now().unwrap());
    let mut transform_stamped = vec![];
    let map_hdr = Header {
        stamp: stamp.clone(),
        frame_id: "map".to_string(),
    };
    let odom_hdr = Header {
        stamp: stamp.clone(),
        frame_id: "odom".to_string(),
    };
    let gimbal_hdr = Header {
        stamp: stamp.clone(),
        frame_id: "gimbal_link".to_string(),
    };
    let camera_hdr = Header {
        stamp: stamp.clone(),
        frame_id: "camera_link".to_string(),
    };

    gimbal_pose_pub.publish(pose!(gimbal_hdr));
    odom_pose_pub.publish(pose!(odom_hdr));
    camera_pose_pub.publish(pose!(camera_hdr));

    add_tf_frame!(
        transform_stamped,
        map_hdr.clone(),
        "odom",
        gimbal.translation(),
        Quat::IDENTITY
    );
    add_tf_frame!(
        transform_stamped,
        odom_hdr.clone(),
        "gimbal_link",
        Vec3::ZERO,
        gimbal.rotation()
    );
    let cam_rel = cam_transform.reparented_to(gimbal.into_inner());
    add_tf_frame!(
        transform_stamped,
        gimbal_hdr.clone(),
        "camera_link",
        cam_rel.translation,
        cam_rel.rotation
    );
    add_tf_frame!(
        transform_stamped,
        camera_hdr.clone(),
        "camera_optical_frame",
        Vec3::ZERO,
        Quat::from_euler(EulerRot::ZYX, -PI / 2.0, PI, PI / 2.0)
    );
    for (transform, rune) in runes {
        add_tf_frame!(
            transform_stamped,
            map_hdr.clone(),
            format!("power_rune_{:?}", rune.mode)
                .to_string()
                .to_lowercase(),
            transform.compute_transform()
        );
    }
    for (target_transform, target, name) in targets {
        if !name.contains("_ACTIVATED") {
            continue;
        }
        if let Ok((_rune_transform, rune)) = runes.get(target.1) {
            add_tf_frame!(
                transform_stamped,
                Header {
                    stamp: stamp.clone(),
                    frame_id: format!("power_rune_{:?}", rune.mode)
                        .to_string()
                        .to_lowercase(),
                },
                format!("power_rune_{:?}_{:?}", rune.mode, target.0)
                    .to_string()
                    .to_lowercase(),
                target_transform.reparented_to(_rune_transform)
            );
        }
    }
    tf_publisher.publish(TFMessage {
        transforms: transform_stamped,
    });
}

/// 将“ROS 对齐世界系”里的 yaw/pitch 命令，转换成 Bevy 世界系里的全局旋转。
///
/// 背景：
/// - /tf 发布时调用了 transform()，其中对位姿做了 M_ALIGN_MAT3 对齐
/// - 算法侧通过 /tf 看到的是 ROS 对齐后的世界系
/// - 因此 /fyt_cmd 发回来的 yaw/pitch 也应该按 ROS 对齐世界系解释
/// - 这里将 ROS 世界系旋转反变换回 Bevy 世界系旋转，再用于控制云台
fn ros_yaw_pitch_to_bevy_global_quat(yaw_ros: f32, pitch_ros: f32) -> Quat {
    let q_ros = Quat::from_euler(EulerRot::YXZ, -yaw_ros, pitch_ros, 0.0);

    let align_quat = Quat::from_mat3(&M_ALIGN_MAT3);

    // transform() 里是：q_ros = A * q_bevy * A^-1
    // 所以反变换：q_bevy = A^-1 * q_ros * A
    align_quat.inverse() * q_ros * align_quat
}

fn apply_fyt_cmd_to_gimbal(
    cmd_state: Res<FytCommandState>,
    chassis_global: Single<
        &GlobalTransform,
        (With<LocalInfantry>, With<InfantryRoot>, Without<InfantryGimbal>),
    >,
    gimbal: Single<
        (&mut Transform, &mut InfantryGimbal),
        (With<LocalInfantry>, Without<InfantryChassis>),
    >,
) {
    let cmd = *cmd_state.0.lock().unwrap();
    if !cmd.received || !cmd.control {
        return;
    }

    // 旧版：
    // let desired_global = Quat::from_euler(EulerRot::YXZ, cmd.yaw, cmd.pitch, 0.0);

    // 新版：把 ROS 世界系命令转换回 Bevy 世界系
    let desired_global = ros_yaw_pitch_to_bevy_global_quat(cmd.yaw, cmd.pitch);

    let local_rotation = chassis_global.rotation().inverse() * desired_global;
    let (local_yaw, pitch, _) = local_rotation.to_euler(EulerRot::YXZ);

    let (mut gimbal_transform, mut gimbal_data) = gimbal.into_inner();
    gimbal_data.local_yaw = local_yaw;
    gimbal_data.pitch = pitch.clamp(-0.785, 0.785);
    gimbal_transform.rotation =
        Quat::from_euler(EulerRot::YXZ, gimbal_data.local_yaw, gimbal_data.pitch, 0.0);
}

fn launch_projectile_from_fyt(
    time: Res<Time>,
    mut cooldown: ResMut<Cooldown>,
    mut commands: Commands,
    setting: Res<ProjectileSetting>,
    infantry: Single<
        (&Transform, &LinearVelocity, &AngularVelocity),
        (With<InfantryRoot>, With<LocalInfantry>),
    >,
    gimbal: Single<
        (&GlobalTransform, &InfantryGimbal),
        (With<LocalInfantry>, Without<InfantryChassis>),
    >,
    launch_offset: Single<&InfantryLaunchOffset, With<LocalInfantry>>,
    cmd_state: Res<FytCommandState>,
) {
    cooldown.0.tick(time.delta());

    let cmd = *cmd_state.0.lock().unwrap();
    if !cmd.received || !cmd.control || !cmd.shoot || !cooldown.0.is_finished() {
        return;
    }
    cooldown.0.reset();

    let direction = (gimbal.0.rotation() * launch_offset.0.rotation)
        .mul_vec3(Vec3::Y)
        .normalize_or_zero();
    if direction == Vec3::ZERO {
        return;
    }

    let vel = infantry.1.0 + direction * 25.0;

    commands.spawn((
        RigidBody::Dynamic,
        Collider::sphere(44.5 * 0.001 / 2.0),
        Mass(44.5 * 0.001),
        Friction::new(1.1),
        Restitution::ZERO,
        LinearDamping(0.05),
        CollisionLayers::new(
            GameLayer::ProjectileSelf,
            [
                GameLayer::Default,
                GameLayer::Vehicle,
                GameLayer::ProjectileSelf,
                GameLayer::ProjectileOther,
                GameLayer::Environment,
            ],
        ),
        Mesh3d(setting.0.clone()),
        MeshMaterial3d(setting.1.clone()),
        LinearVelocity(vel),
        AngularVelocity(infantry.2.0),
        Transform::IDENTITY.with_translation(
            infantry.0.translation + (gimbal.0.rotation() * launch_offset.0.translation),
        ),
        Projectile,
    ));
}

fn cleanup_ros2_system(
    mut exit: MessageReader<AppExit>,
    stop_signal: Res<StopSignal>,
    mut handle_res: ResMut<SpinThreadHandle>,
) {
    if exit.read().len() > 0 {
        stop_signal.0.store(true, Ordering::Release);
        if let Some(handle) = handle_res.0.take() {
            info!("Waiting for ROS 2 spin thread to join...");
            match handle.join() {
                Ok(_) => info!("ROS 2 thread successfully joined. Safe to exit."),
                Err(_) => error!("WARNING: ROS 2 thread panicked or failed to join."),
            }
        }
    }
}

#[derive(Default)]
pub struct ROS2Plugin {}

impl Plugin for ROS2Plugin {
    fn build(&self, app: &mut App) {
        let mut node = Node::create(Context::create().unwrap(), "simulator", "robomaster").unwrap();
        let signal_arc = Arc::new(AtomicBool::new(false));

        let fyt_cmd_state = spawn_fyt_cmd_listener(&mut node, signal_arc.clone());

        publisher!(
            signal_arc,
            app,
            node,
            GlobalTransformTopic,
            GimbalPoseTopic,
            OdomPoseTopic,
            CameraPoseTopic
        );
        let camera_info = Arc::new(publisher!(signal_arc, node, CameraInfoTopic));
        let image_raw = Arc::new(publisher!(signal_arc, node, ImageRawTopic));
        let image_compressed = Arc::new(publisher!(signal_arc, node, ImageCompressedTopic));

        let clock = arc_mutex!(Clock::create(SystemTime).unwrap());

        app.insert_resource(RoboMasterClock(clock.clone()))
            .insert_resource(StopSignal(signal_arc.clone()))
            .insert_resource(fyt_cmd_state)
            .add_plugins(RosCapturePlugin {
                config: CaptureConfig {
                    width: 1440,
                    height: 1080,
                    texture_format: TextureFormat::bevy_default(),
                    fov_y: PI / 180.0 * 45.0,
                },
                context: RosCaptureContext {
                    clock,
                    camera_info,
                    image_raw,
                    image_compressed,
                },
            })
            .add_systems(Last, cleanup_ros2_system)
            .add_systems(
                Update,
                (
                    apply_fyt_cmd_to_gimbal.after(TransformSystems::Propagate),
                    launch_projectile_from_fyt.after(apply_fyt_cmd_to_gimbal),
                    capture_rune.after(TransformSystems::Propagate),
                ),
            )
            .insert_resource(SpinThreadHandle(Some(thread::spawn(move || {
                while !signal_arc.load(Ordering::Acquire) {
                    node.spin_once(Duration::from_millis(1000));
                }
            }))));
    }
}