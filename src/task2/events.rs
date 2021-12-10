use crate::{events::log_event, msgs::detection_msgs::DetectedObject};

use super::drone_state::Qr;

/// Событие происходит, когда дрон находит новый QR-код
#[derive(Debug, Clone, PartialEq)]
pub struct QrFound {
    pub qr: Qr,
    pub index: usize,
}

impl QrFound {
    pub fn new(qr: Qr, index: usize) -> QrFound {
        log_event(&format!("QrFound ({:?})", &qr));
        QrFound { qr, index }
    }
}

/// Событие происходит, когда дрон находит проход
#[derive(Debug, Clone, PartialEq)]
pub struct HoleFound {
    pub hole: DetectedObject,
}

impl HoleFound {
    pub fn new(hole: DetectedObject) -> HoleFound {
        log_event("HoleFound");
        HoleFound { hole }
    }
}

/// Событие происходит, когда дрон долетел до точки посадки
#[derive(Debug, Clone, PartialEq)]
pub struct FlewNearLandingPoint {}

impl FlewNearLandingPoint {
    pub fn new() -> FlewNearLandingPoint {
        log_event("FlewNearLandingPoint");
        FlewNearLandingPoint {}
    }
}

/// Событие происходит, когда дрон пролетает сквозь проём
#[derive(Debug, Clone, PartialEq)]
pub struct FlewThroughHole {
    /// Индекс QR-кода, связанного с проёмом
    pub qr_index: usize,
}

impl FlewThroughHole {
    pub fn new(qr_index: usize) -> FlewThroughHole {
        log_event("FlewThroughHole");
        FlewThroughHole { qr_index }
    }
}
