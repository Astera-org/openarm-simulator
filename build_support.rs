use sha2::{Digest, Sha256};
use std::{
    fs::{self, File},
    path::{Path, PathBuf},
    process::Command,
};

pub fn cache(source: &Path, fallback: &Path) -> PathBuf {
    if let Ok(output) = Command::new("git")
        .current_dir(source)
        .args(["rev-parse", "--path-format=absolute", "--git-common-dir"])
        .output()
        && output.status.success()
    {
        let common = PathBuf::from(String::from_utf8(output.stdout).unwrap().trim());
        return common.parent().unwrap().join("target/openarm-assets");
    }
    // Source archives have no Git metadata; Cargo's output directory remains usable.
    fallback.join("openarm-assets")
}

pub fn fetch(cache: &Path, url: &str, checksum: &str, members: &[&str]) -> PathBuf {
    fs::create_dir_all(cache).expect("create dependency cache");
    let destination = cache.join(checksum);
    let lock = File::options()
        .create(true)
        .append(true)
        .open(cache.join(format!("{checksum}.lock")))
        .expect("open dependency cache lock");
    lock.lock().expect("lock dependency cache");
    if destination.is_dir() {
        return destination;
    }
    eprintln!("Downloading {url}");
    let temporary = tempfile::tempdir_in(cache).expect("create download directory");
    let archive = temporary.path().join("archive.tar.gz");
    let status = Command::new("curl")
        .args([
            "--fail",
            "--location",
            "--silent",
            "--show-error",
            "--retry",
            "3",
        ])
        .arg(url)
        .arg("--output")
        .arg(&archive)
        .status()
        .expect("run curl to download dependency");
    assert!(status.success(), "download failed: {url}");
    assert_eq!(
        format!("{:x}", Sha256::digest(fs::read(&archive).unwrap())),
        checksum,
        "checksum mismatch: {url}"
    );
    let unpacked = temporary.path().join("unpacked");
    fs::create_dir(&unpacked).unwrap();
    let status = Command::new("tar")
        .args(["--extract", "--gzip", "--strip-components=1", "--file"])
        .arg(&archive)
        .arg("--directory")
        .arg(&unpacked)
        .args(members)
        .status()
        .expect("run tar to extract dependency");
    assert!(status.success(), "extract failed: {url}");
    // Only publish complete, verified assets; concurrent worktrees share the lock.
    fs::rename(unpacked, &destination).expect("publish dependency cache entry");
    destination
}
