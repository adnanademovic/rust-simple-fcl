extern crate bindgen;
#[cfg(not(feature = "skip_cc"))]
extern crate cc;

use std::env;
use std::path::PathBuf;

#[cfg(not(feature = "skip_cc"))]
fn build_code() {
    println!("cargo:rustc-link-lib=boost_system");
    println!("cargo:rustc-link-lib=fcl");

    cc::Build::new()
        .cpp(true)
        .file("wrapper.cpp")
        .compile("wrapper.a");
}

#[cfg(feature = "skip_cc")]
fn build_code() {}

fn main() {
    build_code();

    let bindings = bindgen::Builder::default()
        .header("wrapper.h")
        .generate()
        .expect("Unable to generate bindings");

    let out_path = PathBuf::from(env::var("OUT_DIR").unwrap());
    bindings
        .write_to_file(out_path.join("bindings.rs"))
        .expect("Couldn't write bindings!");
}
