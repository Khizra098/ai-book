import React from 'react';
import Layout from '@theme/Layout';

function ProfilePage() {
  return (
    <Layout title="User Profile" description="View and edit your profile information">
      <div className="container margin-vert--lg">
        <div className="row">
          <div className="col col--8 col--offset-2">
            <h1>Your Profile</h1>
            <div className="card">
              <div className="card__body">
                <div className="margin-bottom--lg">
                  <h3>Profile Information</h3>
                  <p><strong>Name:</strong> John Doe</p>
                  <p><strong>Email:</strong> john.doe@example.com</p>
                  <p><strong>Member since:</strong> December 2024</p>
                </div>

                <div className="button-group button-group--block">
                  <button className="button button--primary">Edit Profile</button>
                  <button className="button button--secondary">Change Password</button>
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </Layout>
  );
}

export default ProfilePage;