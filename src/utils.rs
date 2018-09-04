use core::ops::{Deref, DerefMut};

/// Can be used to wrap a single value as Sync even though it isn't, making it the owner's
/// reponsibility to handle synchronization.
pub struct SyncWrapper<T>(T);

unsafe impl<T> Send for SyncWrapper<T> { }
unsafe impl<T> Sync for SyncWrapper<T> { }

impl<T> SyncWrapper<T> {
    pub unsafe fn new(value: T) -> SyncWrapper<T> {
        SyncWrapper(value)
    }
}

impl<T> Deref for SyncWrapper<T> {
    type Target = T;

    fn deref(&self) -> &T {
        &self.0
    }
}

impl<T> DerefMut for SyncWrapper<T> {
    fn deref_mut(&mut self) -> &mut T {
        &mut self.0
    }
}