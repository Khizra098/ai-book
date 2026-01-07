import React from 'react';
import Layout from '@theme/Layout';

function SettingsPage() {
  return (
    <Layout title="User Settings" description="Manage your account settings">
      <div className="container margin-vert--lg">
        <div className="row">
          <div className="col col--8 col--offset-2">
            <h1>Account Settings</h1>
            <div className="card">
              <div className="card__body">
                <div className="margin-bottom--lg">
                  <h3>Preferences</h3>
                  <div className="form-group">
                    <label>Notification Settings</label>
                    <div className="form-checkbox">
                      <input type="checkbox" id="email-notifications" defaultChecked />
                      <label htmlFor="email-notifications">Email Notifications</label>
                    </div>
                    <div className="form-checkbox">
                      <input type="checkbox" id="push-notifications" defaultChecked />
                      <label htmlFor="push-notifications">Push Notifications</label>
                    </div>
                  </div>
                </div>

                <div className="margin-bottom--lg">
                  <h3>Security</h3>
                  <div className="button-group button-group--block">
                    <button className="button button--secondary">Change Password</button>
                    <button className="button button--secondary">Two-Factor Authentication</button>
                  </div>
                </div>

                <div className="margin-bottom--lg">
                  <h3>Privacy</h3>
                  <div className="form-group">
                    <label>Profile Visibility</label>
                    <select className="form-control">
                      <option>Public</option>
                      <option>Private</option>
                      <option>Friends Only</option>
                    </select>
                  </div>
                </div>

                <div className="button-group button-group--block">
                  <button className="button button--primary">Save Changes</button>
                  <button className="button button--default">Cancel</button>
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </Layout>
  );
}

export default SettingsPage;