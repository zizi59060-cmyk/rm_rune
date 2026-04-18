#[macro_export]
macro_rules! arc_mutex {
    ($elem:expr) => {
        ::std::sync::Arc::new(::std::sync::Mutex::new($elem))
    };
}
