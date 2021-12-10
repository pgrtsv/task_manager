use crate::{events::log_event, msgs::detection_msgs::DetectedObject};

/// Событие происходит однократно, когда дрон находит вход в исследуемое здание (первый найденный проём)
#[derive(Debug, Clone, PartialEq)]
pub struct EntryFound {
    pub entry: DetectedObject,
}

impl EntryFound {
    pub fn new(entry: DetectedObject) -> EntryFound {
        log_event("EntryFound");
        EntryFound { entry }
    }
}

/// Событие происходит, когда дрон впервые влетает в исследуемое здание через окно
#[derive(Debug, Clone, PartialEq)]
pub struct FlewInsideBuilding {}

impl FlewInsideBuilding {
    pub fn new() -> FlewInsideBuilding {
        log_event("FlewInsideBuilding");
        FlewInsideBuilding {}
    }
}

/// Событие происходит, когда дрон вернулся на исходную точку
#[derive(Debug, Clone, PartialEq)]
pub struct FlewNearStartPoint {}

impl FlewNearStartPoint {
    pub fn new() -> FlewNearStartPoint {
        log_event("FlewNearStartPoint");
        FlewNearStartPoint {}
    }
}

/// Событие происходит, когда дрон нашел все кубы
#[derive(Debug, Clone, PartialEq)]
pub struct FoundAllCubes {}

impl FoundAllCubes {
    pub fn new() -> FoundAllCubes {
        log_event("FoundAllCubes");
        FoundAllCubes {}
    }
}
