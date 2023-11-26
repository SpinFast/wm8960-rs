use std::env;
use std::fs;
use std::io;
use std::path::PathBuf;
use std::str::FromStr;

use proc_macro2::TokenStream;
use regex::Regex;

// Tools to generate nice register access from yaml descriptions
use chiptool::generate::CommonModule;
use chiptool::{generate, ir, transform};

fn main() -> io::Result<()> {
    let out_dir = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    let yaml_path = PathBuf::from("wm8960.yaml");

    let mut regmap_out_path = PathBuf::from(&out_dir);
    regmap_out_path.push("regmap.rs");

    let mut reset_out_path = PathBuf::from(&out_dir);
    reset_out_path.push("reset.rs");

    fs::write(
        out_dir.join("common.rs"),
        chiptool::generate::iface::COMMON_MODULE,
    )?;

    let data = fs::read(yaml_path.clone())?;
    println!("decoding yaml");
    let mut ir: ir::IR = serde_yaml::from_slice(&data).unwrap();
    println!("read in yaml");
    transform::Sanitize {}.run(&mut ir).unwrap();

    transform::sort::Sort {}.run(&mut ir).unwrap();

    let gen_opts = generate::Options {
        common_module: CommonModule::External(TokenStream::from_str("crate::common").unwrap()),
    };

    let mut tmp = String::new();
    tmp += "pub(crate) const RESET: [u16; 56] = [\n";

    ir.blocks.iter().for_each(|(_block_key, block_item)| {
        let mut i = 0;
        for reg in block_item.items.iter() {
            let byte_offset = reg.byte_offset;
            while i < byte_offset {
                tmp += "0,\n";
                i += 1;
            }
            if let chiptool::ir::BlockItemInner::Register(ref reg) = reg.inner {
                println!("reset value at offset {} is {:x}", byte_offset, reg.reset);
                tmp += &format!("0b{:b},\n", reg.reset);
                i += 1;
            }
        }
    });

    tmp += "];";

    let ts = TokenStream::from_str(&tmp).unwrap();
    fs::write(reset_out_path, ts.to_string())?;

    let data = generate::iface::render(&ir, &gen_opts).unwrap().to_string();

    // Some sane newline additions
    let data = data.replace("] ", "]\n");

    // Replace inner attributes (not allowed)
    let data = Regex::new("# *! *\\[.*\\]").unwrap().replace_all(&data, "");

    println!("generating {:?} from {:?}", &regmap_out_path, yaml_path);
    fs::write(&regmap_out_path, data.to_string())?;

    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed=src/common.rs");
    println!("cargo:rerun-if-changed=wm8960.yaml");
    println!("cargo:rerun-if-changed={}", yaml_path.display());

    Ok(())
}
