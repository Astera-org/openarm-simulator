#[path = "../build_support.rs"]
mod build_support;

use sha2::{Digest, Sha256};
use std::{fs, path::Path, process::Command, thread};

fn git(directory: &Path, args: &[&str]) {
    let output = Command::new("git")
        .current_dir(directory)
        .args(args)
        .output()
        .unwrap();
    assert!(
        output.status.success(),
        "{}",
        String::from_utf8_lossy(&output.stderr)
    );
}

#[test]
fn worktrees_share_verified_assets() {
    let temporary = tempfile::tempdir().unwrap();
    let base = temporary.path();
    git(base, &["init", "repo"]);
    let repo = base.join("repo");
    git(
        &repo,
        &[
            "-c",
            "user.name=Test",
            "-c",
            "user.email=test@example.invalid",
            "commit",
            "--no-gpg-sign",
            "--allow-empty",
            "-m",
            "fixture",
        ],
    );
    git(
        &repo,
        &["worktree", "add", "--detach", "../linked worktree"],
    );
    git(base, &["clone", "--bare", "repo", "bare.git"]);
    git(
        &base.join("bare.git"),
        &["worktree", "add", "--detach", "../bare worktree"],
    );

    let fallback = base.join("out");
    assert_eq!(
        build_support::cache(base, &fallback),
        fallback.join("openarm-assets")
    );
    for (repository, worktree, expected) in [
        (
            repo.clone(),
            base.join("linked worktree"),
            repo.join("target/openarm-assets"),
        ),
        (
            base.join("bare.git"),
            base.join("bare worktree"),
            base.join("target/openarm-assets"),
        ),
    ] {
        let first = build_support::cache(&repository, &fallback);
        let second = build_support::cache(&worktree, &fallback);
        assert_eq!(first, second);
        assert_eq!(first, expected);
    }

    let cache = build_support::cache(&base.join("repo"), &fallback);
    let linked_cache = build_support::cache(&base.join("linked worktree"), &fallback);
    fs::create_dir(base.join("payload")).unwrap();
    fs::write(base.join("payload/value"), "verified fixture").unwrap();
    let archive = base.join("fixture.tar.gz");
    assert!(
        Command::new("tar")
            .current_dir(base)
            .args(["-czf", "fixture.tar.gz", "payload"])
            .status()
            .unwrap()
            .success()
    );
    let checksum = format!("{:x}", Sha256::digest(fs::read(&archive).unwrap()));
    let url = format!("file://{}", archive.display());
    let paths = thread::scope(|scope| {
        let a = scope.spawn(|| build_support::fetch(&cache, &url, &checksum, &[]));
        let b = scope.spawn(|| build_support::fetch(&linked_cache, &url, &checksum, &[]));
        [a.join().unwrap(), b.join().unwrap()]
    });
    assert_eq!(paths[0], paths[1]);
    assert_eq!(
        fs::read_to_string(paths[0].join("value")).unwrap(),
        "verified fixture"
    );
    let wrong_checksum = "0".repeat(64);
    assert!(
        std::panic::catch_unwind(|| build_support::fetch(&cache, &url, &wrong_checksum, &[]))
            .is_err()
    );
    assert!(!cache.join(wrong_checksum).exists());
    fs::remove_file(archive).unwrap();
    // The source is gone: the second worktree must reuse the completed cache entry.
    assert_eq!(
        build_support::fetch(&linked_cache, &url, &checksum, &[]),
        paths[0]
    );
}
