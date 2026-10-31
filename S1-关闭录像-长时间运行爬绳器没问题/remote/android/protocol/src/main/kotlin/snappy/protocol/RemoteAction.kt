package snappy.protocol

enum class RemoteAction(val wireValue: String) {
    ControlAcquire("control.acquire"),
    ControlArm("control.arm"),
    ControlRelease("control.release"),
    ControlHeartbeat("control.heartbeat"),
    ControlSafeHold("control.safe_hold"),
    ControlEstop("control.estop"),
    FaultClear("fault.clear"),

    BrushForward("brush.forward"),
    BrushReverse("brush.reverse"),
    BrushStop("brush.stop"),

    BrushPushExtend("brush_push.extend"),
    BrushPushRetract("brush_push.retract"),
    BrushPushExtendTimed("brush_push.extend_timed"),
    BrushPushRetractTimed("brush_push.retract_timed"),
    BrushPushStop("brush_push.stop"),

    ScraperPushExtend("scraper_push.extend"),
    ScraperPushRetract("scraper_push.retract"),
    ScraperPushExtendTimed("scraper_push.extend_timed"),
    ScraperPushRetractTimed("scraper_push.retract_timed"),
    ScraperPushStop("scraper_push.stop"),

    // Legacy single-push commands. The new UI uses brush/scraper push commands.
    PushExtend("push.extend"),
    PushRetract("push.retract"),
    PushStop("push.stop"),

    FanPrepare("fan.prepare"),
    FanEnable("fan.enable"),
    FanDisable("fan.disable"),
    FanLeftSet("fan.left_set"),
    FanRightSet("fan.right_set"),
    FanBothSet("fan.both_set"),
    FanStop("fan.stop"),

    ClimberAscend("climber.ascend"),
    ClimberDescend("climber.descend"),
    ClimberStop("climber.stop"),

    VideoStart("video.start"),
    VideoStop("video.stop"),
    VideoSwitchCamera("video.switch_camera"),
    VideoToggle("video.toggle"),

    TelemetryEnable("telemetry.enable"),
    TelemetryDisable("telemetry.disable"),

    GatewayCapabilities("gateway.capabilities");

    companion object {
        fun fromWireValue(value: String): RemoteAction? {
            return values().firstOrNull { it.wireValue == value }
        }
    }
}