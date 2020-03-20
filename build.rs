///! Build script for v1026-demo-app
///!
///! Gets git version info.
///!
///! Based on https://github.com/rustyhorde/vergen, which we use under the MIT
///! licence (http://opensource.org/licenses/MIT).
use std::process::Command;

fn main() {
	let sha = run_command(Command::new("git").args(&["rev-parse", "--short", "HEAD"]));
	let version = run_command(Command::new("git").args(&["describe", "--dirty"]));

	println!("cargo:rustc-env=BUILD_GIT_HASH={}", sha);
	println!("cargo:rustc-env=BUILD_GIT_VERSION={}", version);
	println!("cargo:rerun-if-env-changed=V1026_UNIT_ID");
	println!("cargo:rerun-if-env-changed=V1026_UNIT_API_KEY");
}

fn run_command(command: &mut Command) -> String {
	if let Ok(o) = command.output() {
		if o.status.success() {
			return String::from_utf8_lossy(&o.stdout).trim().to_owned();
		}
	}
	return "UNKNOWN".to_owned();
}
