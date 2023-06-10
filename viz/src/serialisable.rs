pub trait Serialise {
    fn serialise(&self) -> Vec<u8>;
}

pub trait Deserialise<T> {
    fn deserialise(&self) -> Option<T>;
}
