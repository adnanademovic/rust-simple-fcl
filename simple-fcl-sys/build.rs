extern crate bindgen;
extern crate cc;

use std::env;
use std::path::PathBuf;

fn main() {
    println!("cargo:rustc-link-lib=boost_system");
    println!("cargo:rustc-link-lib=fcl");

    cc::Build::new()
        .cpp(true)
        .file("wrapper.cpp")
        .compile("wrapper.a");

    let bindings = bindgen::Builder::default()
        .header("wrapper.h")
        .generate()
        .expect("Unable to generate bindings");

    let out_path = PathBuf::from(env::var("OUT_DIR").unwrap());
    bindings
        .write_to_file(out_path.join("bindings.rs"))
        .expect("Couldn't write bindings!");
}
